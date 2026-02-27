const express = require('express');
const http = require('http');
const { Server } = require('socket.io');
const { spawn } = require('child_process');
const path = require('path');
const pty = require('node-pty');

const app = express();
const server = http.createServer(app);
const io = new Server(server);

const PORT = 8009;

// ==========================================
// ROBOT CONFIGURATION (matching validation script)
// ==========================================
const ROBOT_CONFIG = {
    username: 'zippy',
    password: 'zippy',
    ipPrefix: '10.30.72.',
    offsetIP: 60
};

// Helper function to calculate robot IP
function getRobotIP(robotNumber) {
    const ip = parseInt(robotNumber) + ROBOT_CONFIG.offsetIP;
    return ROBOT_CONFIG.ipPrefix + ip;
}

// Helper function to get robot credentials
function getRobotCredentials() {
    return {
        username: ROBOT_CONFIG.username,
        password: ROBOT_CONFIG.password
    };
}

// Redirect root to login page
app.get('/', (req, res) => {
    res.redirect('/login.html');
});

// Serve static files from 'public' folder
app.use(express.static(path.join(__dirname, 'public')));

// Prevent concurrent execution
let isCommandRunning = false;
let currentProcess = null; // ✅ ADDED (was missing)
let currentFlag = null;    // Track which flag is running (for teleop stop message)

io.on('connection', (socket) => {
    console.log('A user connected');

    // Send initial status
    socket.emit('status', { isRunning: isCommandRunning });

    socket.on('run-command', (data) => {
        if (isCommandRunning) {
            socket.emit('output', 'Error: A command is already running. Please wait.\n');
            return;
        }

        const { robotType, flag, robotNumber } = data;

        if (!robotNumber || !flag) {
            socket.emit('output', 'Error: Missing robot number or flag.\n');
            return;
        }

        // Validate robot number (numeric)
        if (!/^\d+$/.test(robotNumber)) {
            socket.emit('output', 'Error: Robot number must be numeric.\n');
            return;
        }

        // Determine script based on robotType
        let scriptName = '';
        if (robotType === 'Zippy6') {
            scriptName = 'zippy6_validation.sh';
        } else if (robotType === 'Zippy10') {
            scriptName = 'zippy10_validation.sh';
        } else {
            socket.emit('output', 'Error: Unknown robot type.\n');
            return;
        }

        const scriptPath = path.join(__dirname, 'scripts', scriptName);

        // Telemetry Flags (High-rate ROS topics that stream continuously)
        const TELEMETRY_FLAGS = ['-d', '-D', '-IM', '-RO', '-SY', '-IN', '-SE', '-BP', '-CR'];
        const isTelemetry = TELEMETRY_FLAGS.includes(flag);

        let localBuffer = '';
        let latestMessage = ''; // Store only the most recent message

        const throttleInterval = setInterval(() => {
            if (isTelemetry) {
                if (latestMessage) {
                    io.emit('output', latestMessage + '\n---\n'); // Send latest only
                    // We don't clear latestMessage here to keep UI showing last state 
                    // until new one arrives, but for terminal feel, sending once is better.
                    latestMessage = '';
                }
            } else if (localBuffer.length > 0) {
                io.emit('output', localBuffer);
                localBuffer = '';
            }
        }, 150); // Slightly faster interval (150ms) for telemetry

        isCommandRunning = true;
        currentFlag = flag;  // Track which flag is running
        io.emit('status', { isRunning: true });
        io.emit('output', `> Executing: ./${scriptName} ${flag} ${robotNumber}\n`);

        // Use stdbuf -o0 to disable output buffering in the spawned process
        const child = spawn('stdbuf', ['-o0', 'bash', scriptPath, flag, String(robotNumber)], {
            detached: true,
            env: { ...process.env }
        });

        currentProcess = child;

        child.stdout.on('data', (chunk) => {
            const data = chunk.toString();
            if (isTelemetry) {
                // ROS1 rostopic echo uses '---' as message separator
                // We want the text between the last two '---' or after the last '---'
                localBuffer += data;
                const messages = localBuffer.split('\n---\n');
                if (messages.length > 1) {
                    // Latest complete message is the second to last element
                    let candidate = messages[messages.length - 2];

                    // FIELD FILTERING: For raw_odom (-RO), keep only essentials
                    if (flag === '-RO' && candidate.includes('pose:')) {
                        const posePart = candidate.match(/pose:[\s\S]*?twist:/);
                        if (posePart) candidate = posePart[0].replace('twist:', '').trim();
                    }

                    latestMessage = candidate;
                    localBuffer = messages[messages.length - 1]; // Keep leftovers
                }
                // Limit internal buffer to 100KB to avoid memory creep
                if (localBuffer.length > 100000) localBuffer = localBuffer.slice(-100000);
            } else {
                localBuffer += data;
                if (localBuffer.length > 1000000) localBuffer = localBuffer.slice(-1000000);
            }
        });

        child.stderr.on('data', (chunk) => {
            // Don't prefix with ERR: since many tools (like rostopic) print normal messages to stderr
            localBuffer += chunk.toString();
            if (localBuffer.length > 1000000) localBuffer = localBuffer.slice(-1000000);
        });

        child.on('close', (code) => {
            clearInterval(throttleInterval);
            // Final flush
            if (isTelemetry && latestMessage) {
                io.emit('output', latestMessage + '\n');
            } else if (localBuffer.length > 0) {
                io.emit('output', localBuffer);
            }

            isCommandRunning = false;
            currentProcess = null;
            currentFlag = null;  // Reset flag tracking
            io.emit('status', { isRunning: false });

            if (code === null) {
                io.emit('output', `\n> Command stopped manually.\n`);
            } else {
                io.emit('output', `\n> Command finished with exit code ${code}\n`);
            }
        });

        child.on('error', (err) => {
            clearInterval(throttleInterval);
            isCommandRunning = false;
            currentProcess = null;
            io.emit('status', { isRunning: false });
            io.emit('output', `\n> Failed to start command: ${err.message}\n`);
        });
    });

    socket.on('stop-command', () => {
        if (currentProcess) {
            try {
                // Show appropriate message based on what's running
                if (currentFlag === '-N') {
                    io.emit('output', '\n> Teleop stopped by user\n');
                } else {
                    io.emit('output', '\n> Stopping command...\n');
                }
                process.kill(-currentProcess.pid, 'SIGTERM'); // Kill process group
            } catch (e) {
                socket.emit('output', `\n> Error stopping command: ${e.message}\n`);
            }
        }
    });

    // ==========================================
    // TELEOP TERMINAL (Real PTY for keyboard control)
    // ==========================================
    let teleopPty = null;

    socket.on('start-teleop', (data) => {
        const { robotNumber, robotIP, robotPassword } = data;

        if (teleopPty) {
            socket.emit('teleop-output', '\r\n⚠ Teleop session already running. Stop it first.\r\n');
            return;
        }

        console.log(`Starting teleop for robot ${robotNumber} at ${robotIP}`);
        socket.emit('teleop-output', `\r\n🚀 Starting teleop session for Robot ${robotNumber}...\r\n`);
        socket.emit('teleop-output', `📍 Use keys: W=forward, X=backward, A=left, D=right, S=stop\r\n`);
        socket.emit('teleop-output', `─────────────────────────────────────────\r\n`);

        // The command to run
        const sshCommand = `sshpass -p ${robotPassword} ssh -t -o StrictHostKeyChecking=no zippy@${robotIP} "sudo docker exec -it zippy bash -c 'source /home/zippy/zippy_ws/install/setup.bash && rosrun turtlebot3_teleop turtlebot3_teleop_key'"`;

        // Create PTY
        teleopPty = pty.spawn('bash', ['-c', sshCommand], {
            name: 'xterm-color',
            cols: 80,
            rows: 24,
            cwd: process.cwd(),
            env: process.env
        });

        teleopPty.onData((data) => {
            socket.emit('teleop-output', data);
        });

        teleopPty.onExit(({ exitCode }) => {
            socket.emit('teleop-output', `\r\n─────────────────────────────────────────\r\n`);
            socket.emit('teleop-output', `⚡ Teleop session ended (exit code: ${exitCode})\r\n`);
            socket.emit('teleop-stopped');
            teleopPty = null;
        });
    });

    socket.on('teleop-input', (data) => {
        if (teleopPty) {
            teleopPty.write(data);
        }
    });

    socket.on('stop-teleop', () => {
        if (teleopPty) {
            socket.emit('teleop-output', '\r\n⏹ Stopping teleop & exiting robot...\r\n');
            // Send Ctrl+C to properly exit the ROS node
            teleopPty.write('\x03');  // Ctrl+C
            // Then send exit to close the shell
            setTimeout(() => {
                if (teleopPty) {
                    teleopPty.write('exit\n');
                }
            }, 300);
            // Wait a moment then kill the PTY
            setTimeout(() => {
                if (teleopPty) {
                    teleopPty.kill('SIGINT');
                    teleopPty = null;
                }
            }, 800);
        }
    });

    socket.on('resize-teleop', (data) => {
        if (teleopPty && data.cols && data.rows) {
            teleopPty.resize(data.cols, data.rows);
        }
    });

    // PING ROBOT (Connection Test)
    // ==========================================
    socket.on('ping-robot', (data) => {
        const { robotNumber, robotIP } = data;

        console.log(`Pinging robot ${robotNumber} at ${robotIP}...`);

        const startTime = Date.now();

        // Use ping command: -c 1 = 1 packet, -W 1 = 1 second timeout
        const pingProcess = spawn('ping', ['-c', '1', '-W', '2', robotIP]);

        let output = '';

        pingProcess.stdout.on('data', (chunk) => {
            output += chunk.toString();
        });

        pingProcess.stderr.on('data', (chunk) => {
            output += chunk.toString();
        });

        pingProcess.on('close', (code) => {
            const responseTime = Date.now() - startTime;

            if (code === 0) {
                // Success - Robot is reachable
                const timeMatch = output.match(/time=([\d.]+)\s*ms/);
                const pingTime = timeMatch ? timeMatch[1] + ' ms' : responseTime + ' ms';

                socket.emit('ping-result', {
                    success: true,
                    robotNumber: robotNumber,
                    robotIP: robotIP,
                    responseTime: pingTime
                });
            } else {
                // Failed - Robot is not reachable
                socket.emit('ping-result', {
                    success: false,
                    robotNumber: robotNumber,
                    robotIP: robotIP,
                    error: 'Host unreachable or timeout'
                });
            }
        });

        pingProcess.on('error', (err) => {
            socket.emit('ping-result', {
                success: false,
                robotNumber: robotNumber,
                robotIP: robotIP,
                error: err.message
            });
        });
    });

    socket.on('disconnect', () => {
        // Clean up teleop PTY on disconnect
        if (teleopPty) {
            teleopPty.kill();
            teleopPty = null;
        }
        console.log('User disconnected');
    });
});

server.listen(PORT, '0.0.0.0', () => {
    console.log(`Server running on http://0.0.0.0:${PORT}`);
});
