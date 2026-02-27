# 🤖 Zippy Robot Control Dashboard

A web-based control panel for **Zippy6** and **Zippy10** robots. Execute Docker commands, monitor ROS topics, control teleop, and manage robot configuration — all from your browser.

## ✨ Features

- **Robot Selection** — Choose between Zippy6 and Zippy10
- **20 Docker/Robot Commands** — SSH, Docker Restart, Inspect, Cleanup, Firmware Update, and more
- **Live ROS Topic Monitoring** — BMS Debug, IMU Data, Raw Odom, Syncmover Info, etc.
- **Teleop Mode** — Real-time keyboard control via xterm.js terminal
- **Conveyor Control** — Forward/Backward conveyor operation
- **Live Terminal Output** — Buffered, auto-scrolling terminal with log download
- **Login Page** — Simple authentication for access control

## 📋 Commands Available

| Flag | Command |
|------|---------|
| `-s` | SSH into Robot |
| `-i` | Inspect Docker Image |
| `-x` | Docker Restart |
| `-u` | Run Setup Script |
| `-p` | Robot Parameters Change |
| `-P` | Parameters & Docker Setup Change |
| `-z` | Add Zip Deb |
| `-v` | Update Syncmover Firmware |
| `-y` | SSH Keygen |
| `-c` | Docker Images Cleanup |
| `-h` | Hostname Change |
| `-d` | Display Rostopics |
| `-S` | SCP Saviour Files |
| `-D` | Display /bms_debug Topic |
| `-IM` | Display /imu/data Topic |
| `-RO` | Display /raw_odom Topic |
| `-SY` | Display /syncmoverinfo Topic |
| `-IN` | Display /info Topic |
| `-SE` | Display /saviour_error Topic |
| `-BP` | Display /barcode_pose_raw Topic |

## 🚀 Quick Start

### Prerequisites
- Node.js 18+
- `sshpass` installed on host
- Network access to robots

### Run Locally

```bash
# Install dependencies
npm install

# Start the server
npm start
```

Open browser: **http://localhost:8009**

### Run with Docker

```bash
# Build and run
docker compose up -d

# Or build manually
docker build -t robot-control-web .
docker run -p 8009:8009 robot-control-web
```

### Save Docker Image

```bash
# Save as tar
docker save robot-control-web:latest -o robot-control-web.tar

# Load from tar
docker load -i robot-control-web.tar
```

## 📁 Project Structure

```
zippy6-ui/
├── server.js                 # Node.js backend (Express + Socket.IO)
├── package.json              # Dependencies
├── Dockerfile                # Docker image config
├── docker-compose.yml        # Docker Compose config
├── start-server.sh           # Quick start script
├── public/                   # Frontend files
│   ├── index.html            # Main dashboard
│   ├── login.html            # Login page
│   ├── script.js             # Frontend logic
│   ├── style.css             # Styles
│   ├── xterm.js              # Terminal emulator
│   ├── xterm.css             # Terminal styles
│   ├── xterm-addon-fit.js    # Terminal fit addon
│   └── assets/               # Robot images
└── scripts/                  # Validation scripts
    ├── zippy6_validation.sh  # Zippy6 commands
    └── zippy10_validation.sh # Zippy10 commands
```

## 🔑 Default Login

- **Username:** `validation`
- **Password:** `validation123`

## 📡 Port

Server runs on port **8009**
