#!/bin/bash
#set -x
sourceName=prakhar.agrawal
sourceIP=10.12.1.150
sourcePassword="Addverb@2025"
password=zippy
IPPrefix=10.12.1.
offsetIP1=150
offsetIP2=150
paramFilePath=/home/prakhar.agrawal/Scripts/zippy10_Mercado/robot_parameters
dockerSetupFilePath=/home/prakhar.agrawal/Scripts/zippy10_Mercado/docker_setup
firmwarepath=/home/prakhar.agrawal/Scripts/zippy10_Mercado/DHL_HOST_CLI
saviourpath=/home/prakhar.agrawal/Scripts/zippy6_Mercado/saviour.zip
id=${1}
hostname="Zippy06-Mercado-"

print_help() {
	echo "Help Manual is:"
    echo "-s - ssh into robot"
    echo "-i - Inspect Docker image"
    echo "-x - Docker restart"
    echo "-u - Run setup script"    
    echo "-p - robot_parameters change"
    echo "-P - robot_parameters and docker setup change"
    echo "-z - Adding zip deb"
    echo "-v - Updating syncmover firmware"
    echo "-y - ssh keygen"
    echo "-c - Docker images cleanup"
    echo "-h - Hostname change"
    echo "-d - Display Rostopics"
    echo "-S - SCP saviour files"
    echo "-D - Display /bms_debug topic"
    echo "-IM - Display /imu/data topic"
    echo "-RO - Display /raw_odom topic"
    echo "-SY - Display /syncmoverinfo topic"
    echo "-IN - Display /info topic"
    echo "-SE - Display /saviour_error topic"
    echo "-BP - Display /barcode_pose_raw topic"
    echo "-SH - Display Error logs from robot"
    echo "-CR - Display /robot_control/cmd_vel topic"
}

case "$1" in
    "") 
        echo "=== Please provide atleast one argument for example $0 -s ==="
        print_help
        exit 1
        ;;
    -s|-i|-x|-u|-P|-p|-z|-v|-y|-c|-h|-d|-S|-D|-IM|-RO|-SY|-IN|-SE|-BP|-SH|-CR)
        setTarget="$1"
        ;;
    *)
        echo "Error: Invalid option '$1'"
        print_help
        exit 1
        ;;
esac

if [[ $# -gt 2 || ($# -eq 2 && $2 =~ ^[0-9]+$) ]]; then
    robotNumbers=()
    stringArgs=()

    for arg in "${@:2}"; do
        if [[ "$arg" =~ ^[0-9]+$ ]]; then
            robotNumbers+=("$arg")
        else
            stringArgs+=("$arg")
        fi
    done

    if [[ ${#stringArgs[@]} -eq 0 ]]; then
        :
    elif [[ ${#stringArgs[@]} -eq 1 ]]; then
        rosbag="${stringArgs[0]}"
    else
        echo "Invalid usage: Too many non-numeric arguments."
        exit 1
    fi

elif [[ $# -le 2 ]]; then

    if [[ $# -eq 2 ]]; then
        if [[ ! $2 =~ ^[0-9]+$ ]]; then
            rosbag="$2"
        else
            echo "❌ Invalid usage: Second argument should be a non-numeric string (e.g., rosbag name)."
            exit 1
        fi
    fi

    echo "Do you want to enter:"
    echo "1. Random robot numbers"
    echo "2. Series of robot numbers"
    read -p "Choose 1 or 2: " choice

    if [[ "$choice" == "1" ]]; then
        read -p "Enter robot numbers (space separated): " -a robotNumbers
    elif [[ "$choice" == "2" ]]; then
        read -p "Enter start number: " start
        read -p "Enter end number: " end
        for ((i=start; i<=end; i++)); do
            robotNumbers+=("$i")
        done
    else
        echo "❌ Invalid choice. Exiting."
        exit 1
    fi
else
    echo "❌ Invalid usage."
    exit 1
fi

for id in "${robotNumbers[@]}"; do
    ((IP = id + offsetIP1))
    robotIP="${IPPrefix}${IP}"
    robotpassword="${password}"
    Id=$(printf "%02d" "$id")
    new_hostname="${hostname}${Id}"
    echo
    echo "=== Working for Robot ID: ${id} with IP: ${robotIP} ==="
    if ! ping -c 1 -W 2 "$robotIP" &> /dev/null; then
        echo "ERROR ❌ : Robot IP $robotIP is not reachable. Skipping robot ID: ${id}"
        continue
    fi

    ssh-keygen -f "$HOME/.ssh/known_hosts" -R "$robotIP" &>/dev/null

    #SSH into robot using (-s)
    if [[ "$setTarget" == "-s" ]]; then
        echo "Connecting via SSH..."
        sshpass -p "$robotpassword" ssh -o StrictHostKeyChecking=no "zippy@$robotIP"

    #Inspect docker image on robot using (-i)
    elif [[ "$setTarget" == "-i" ]]; then
        echo "Inspecting Docker image..."
        command="echo ${robotpassword} | sudo -S docker inspect zippy | grep 'Image' | awk 'NR==1'"
        sshpass -p "$robotpassword" ssh -o StrictHostKeyChecking=no "zippy@$robotIP" "$command"

    #Docker restart using (-x)
    elif [[ "$setTarget" == "-x" ]]; then
        command="echo ${robotpassword} | sudo -S /opt/docker_setup/startup.sh"
        echo "${command}"
        sshpass -p ${robotpassword} ssh -oStrictHostKeyChecking=no zippy@${robotIP} ${command}

    #Run setup script using (-u)
    elif [[ "$setTarget" == "-u" ]]; then
        command="cd /opt/docker_setup/ && echo ${robotpassword} | sudo -S ./setup_robot.sh"
        echo "cd /opt/docker_setup/ && sudo -S ./setup_robot.sh"
        sshpass -p ${robotpassword} ssh -oStrictHostKeyChecking=no zippy@${robotIP} ${command}

    #robot_parameters change using (-p)
    elif [[ "$setTarget" == "-p" ]]; then
        command1="echo ${robotpassword} | sudo -S rm -rf robot_parameters"
        command4="echo ${robotpassword} | sudo -S /opt/docker_setup/startup.sh"
        echo "${command1}"
        sshpass -p ${robotpassword} ssh -oStrictHostKeyChecking=no zippy@${robotIP} ${command1}
        echo "sshpass -p ${robotpassword} scp -r ${paramFilePath} zippy@${robotIP}:/home/zippy/"
        sshpass -p ${robotpassword} scp -r ${paramFilePath} zippy@${robotIP}:/home/zippy/
        echo "${command4}"
        sshpass -p ${robotpassword} ssh -oStrictHostKeyChecking=no zippy@${robotIP} ${command4}
        echo

    #robot_parameters and docker setup change using (-P)
    elif [[ "$setTarget" == "-P" ]]; then
        command1="echo ${robotpassword} | sudo -S docker rm -f zippy"
        command4="echo ${robotpassword} | sudo -S rm -rf robot_parameters /opt/docker_setup"
        command2="cd /home/zippy/ && echo ${robotpassword} | sudo -S mv docker_setup /opt/"
        command3="echo ${robotpassword} | sudo -S /opt/docker_setup/startup.sh"
        echo "${command1}"
        sshpass -p ${robotpassword} ssh -oStrictHostKeyChecking=no zippy@${robotIP} ${command1}
        echo "${command4}"
        sshpass -p ${robotpassword} ssh -oStrictHostKeyChecking=no zippy@${robotIP} ${command4}
        echo "sshpass -p ${robotpassword} scp -r ${paramFilePath} zippy@${robotIP}:/home/zippy/"
        sshpass -p ${robotpassword} scp -r ${paramFilePath} zippy@${robotIP}:/home/zippy/
        echo "sshpass -p ${robotpassword} scp -r ${dockerSetupFilePath} zippy@${robotIP}:/home/zippy/"
        sshpass -p ${robotpassword} scp -r ${dockerSetupFilePath} zippy@${robotIP}:/home/zippy/
        echo "${command2}"
        sshpass -p ${robotpassword} ssh -oStrictHostKeyChecking=no zippy@${robotIP} ${command2}
        #echo "${command3}"
        #sshpass -p ${robotpassword} ssh -oStrictHostKeyChecking=no zippy@${robotIP} ${command3}

    #Adding zip deb using (-z)
    elif [[ "$setTarget" == "-z" ]]; then
        command="echo '${robotpassword}' | sudo -S mv /home/zippy/zip /usr/bin"
        echo "adding zip deb"
        echo "sshpass -p ${robotpassword} scp ${zip_path} zippy@${robotIP}:/home/zippy"
        sshpass -p "${robotpassword}" scp ${zip_path} zippy@${robotIP}:/home/zippy
        echo "${command}"
        sshpass -p ${robotpassword} ssh -oStrictHostKeyChecking=no zippy@${robotIP} ${command}

    #Update syncmover firmware using (-v)
    elif [[ "$setTarget" == "-v" ]]; then
        command1="echo ${robotpassword} | sudo -S docker rm -f \$(docker ps -q)"
        command3='cd ~/DHL_HOST_CLI && echo ${robotpassword} | sudo -S chmod +x ./host_cli && echo ${robotpassword} | sudo -S ./host_cli -v && echo ${robotpassword} | sudo -S ./host_cli -o dual-motor-driver20.bin && echo ${robotpassword} | sudo -S ./host_cli -v'
        command4="echo ${robotpassword} | sudo -S /opt/docker_setup/startup.sh"
        echo "${command1}"
        sshpass -p ${robotpassword} ssh -oStrictHostKeyChecking=no zippy@${robotIP} ${command1}
        echo "sshpass -p ${robotpassword} scp -r ${firmwarepath} zippy@${robotIP}:/home/zippy/"
        sshpass -p ${robotpassword} scp -r ${firmwarepath} zippy@${robotIP}:/home/zippy/
        echo "${command3}"
        sshpass -p ${robotpassword} ssh -oStrictHostKeyChecking=no zippy@${robotIP} ${command3}
        sleep 2
        echo "firmware updated"
        echo "${command4}"
        sshpass -p ${robotpassword} ssh -oStrictHostKeyChecking=no zippy@${robotIP} ${command4}

    #ssh keygen using (-y)
    elif [[ "$setTarget" == "-y" ]]; then
        command1="echo ${robotpassword} | sudo -S rm -f /home/zippy/.ssh/id_ed25519 && ssh-keygen -t ed25519 -f /home/zippy/.ssh/id_ed25519 -N ''"
        command2="ssh-keygen -A"
        command3="echo ${robotpassword} | sudo -S service ssh --full-restart"
        command4="echo ${robotpassword} | sudo -S lvm lvextend -l +100%FREE /dev/ubuntu-vg/ubuntu-lv"
        command5="echo ${robotpassword} | sudo -S resize2fs -p /dev/mapper/ubuntu--vg-ubuntu--lv"
        command6="echo ${robotpassword} | sudo -S systemctl disable systemd-networkd.service systemd-networkd-wait-online.service"
        command7="echo ${robotpassword} | sudo -S systemctl mask systemd-networkd.service systemd-networkd-wait-online.service"
        echo "${command1}"
        sshpass -p ${robotpassword} ssh -oStrictHostKeyChecking=no zippy@${robotIP} ${command1}
        echo "${command2}"
        sshpass -p ${robotpassword} ssh -oStrictHostKeyChecking=no zippy@${robotIP} ${command2}
        echo "${command3}"
        sshpass -p ${robotpassword} ssh -oStrictHostKeyChecking=no zippy@${robotIP} ${command3}
        echo "${command4}"
        sshpass -p ${robotpassword} ssh -oStrictHostKeyChecking=no zippy@${robotIP} ${command4}
        echo "${command5}"
        sshpass -p ${robotpassword} ssh -oStrictHostKeyChecking=no zippy@${robotIP} ${command5}
        echo "${command6}"
        sshpass -p ${robotpassword} ssh -oStrictHostKeyChecking=no zippy@${robotIP} ${command6}
        echo "${command7}"
        sshpass -p ${robotpassword} ssh -oStrictHostKeyChecking=no zippy@${robotIP} ${command7}

    #Docker images cleanup using (-c)
    elif [[ "$setTarget" == "-c" ]]; then
        command1="cd /home/zippy/Robot_Configurator/ && echo ${robotpassword} | sudo -S docker compose up -d"
        command2="echo ${robotpassword} | sudo -S docker system prune -a -f"
        command3="echo ${robotpassword} | sudo -S /opt/docker_setup/startup.sh"
        echo "${command3}"
        sshpass -p ${robotpassword} ssh -oStrictHostKeyChecking=no zippy@${robotIP} ${command3}
        echo "${command1}"
        sshpass -p ${robotpassword} ssh -oStrictHostKeyChecking=no zippy@${robotIP} ${command1}
        echo "${command2}"
        sshpass -p ${robotpassword} ssh -oStrictHostKeyChecking=no zippy@${robotIP} ${command2}

    #Hostname change using (-h)
    elif [[ "$setTarget" == "-h" ]]; then
        command="echo ${robotpassword} | sudo -S hostnamectl set-hostname ${new_hostname}"
        sshpass -p ${robotpassword} ssh -oStrictHostKeyChecking=no zippy@${robotIP} ${command}

    #Display Rostopics using (-d)
    elif [[ "$setTarget" == "-d" ]]; then
        commandA="source /home/zippy/zippy_ws/install/setup.bash"
        commandB="echo ''"
        commandC="echo Topic BMS DATA of Robot no. ${id}"
        commandD="rostopic echo /info -n1 | grep soc"
        commandE="echo ''"
        commandF="echo Topic SyncMover_Info of Robot no. ${id}"
        commandG="rostopic echo /syncmoverinfo -n1"
        commandH="echo ''"
        commandI="echo Topic Info of Robot no. ${id}"
        commandJ="rostopic echo /info -n1"
        commandK="exit"
        dockerCommand="sudo -S docker exec zippy bash -c '${commandA} && ${commandB} && ${commandC} && ${commandD} && ${commandE} && ${commandF} && ${commandG} && ${commandH} && ${commandI} && ${commandJ} && ${commandK}'"
        sshpass -p ${robotpassword} ssh -t -oStrictHostKeyChecking=no zippy@${robotIP} "echo ${robotpassword} | ${dockerCommand}"

    #Display /bms_debug topic using (-D)
    elif [[ "$setTarget" == "-D" ]]; then
        echo "=== Entering Docker container on Robot ${id} (${robotIP}) and reading /bms_debug ==="
        dockerCommand="sudo -S docker exec -i zippy bash -c 'source /home/zippy/zippy_ws/install/setup.bash && echo \"\" && echo \"Topic BMS_DEBUG of Robot no. ${id}\" && rostopic echo /bms_debug'"
        sshpass -p ${robotpassword} ssh -t -oStrictHostKeyChecking=no zippy@${robotIP} "echo ${robotpassword} | ${dockerCommand}"

    #Display /imu/data topic using (-IM)
    elif [[ "$setTarget" == "-IM" ]]; then
        echo "=== Entering Docker container on Robot ${id} (${robotIP}) and reading /imu/data ==="
        dockerCommand="sudo -S docker exec -i zippy bash -c 'source /home/zippy/zippy_ws/install/setup.bash && echo \"\" && echo \"Topic IMU_DATA of Robot no. ${id}\" && rostopic echo /imu/data'"
        sshpass -p ${robotpassword} ssh -t -oStrictHostKeyChecking=no zippy@${robotIP} "echo ${robotpassword} | ${dockerCommand}"

    #Display /raw_odom topic using (-RO)
    elif [[ "$setTarget" == "-RO" ]]; then
        echo "=== Entering Docker container on Robot ${id} (${robotIP}) and reading /raw_odom ==="
        dockerCommand="sudo -S docker exec -i zippy bash -c 'source /home/zippy/zippy_ws/install/setup.bash && echo \"\" && echo \"Topic RAW_ODOM of Robot no. ${id}\" && rostopic echo /raw_odom'"
        sshpass -p ${robotpassword} ssh -t -oStrictHostKeyChecking=no zippy@${robotIP} "echo ${robotpassword} | ${dockerCommand}"

    #Display /syncmoverinfo topic using (-SY)
    elif [[ "$setTarget" == "-SY" ]]; then
        echo "=== Entering Docker container on Robot ${id} (${robotIP}) and reading /syncmoverinfo ==="
        dockerCommand="sudo -S docker exec -i zippy bash -c 'source /home/zippy/zippy_ws/install/setup.bash && echo \"\" && echo \"Topic SYNCMOVERINFO of Robot no. ${id}\" && rostopic echo /syncmoverinfo'"
        sshpass -p ${robotpassword} ssh -t -oStrictHostKeyChecking=no zippy@${robotIP} "echo ${robotpassword} | ${dockerCommand}"

    #Display /info topic using (-IN)
    elif [[ "$setTarget" == "-IN" ]]; then
        echo "=== Entering Docker container on Robot ${id} (${robotIP}) and reading /info ==="
        dockerCommand="sudo -S docker exec -i zippy bash -c 'source /home/zippy/zippy_ws/install/setup.bash && echo \"\" && echo \"Topic INFO of Robot no. ${id}\" && rostopic echo /info'"
        sshpass -p ${robotpassword} ssh -t -oStrictHostKeyChecking=no zippy@${robotIP} "echo ${robotpassword} | ${dockerCommand}"

    #Display /saviour_error topic using (-SE)
    elif [[ "$setTarget" == "-SE" ]]; then
        echo "=== Entering Docker container on Robot ${id} (${robotIP}) and reading /saviour_error ==="
        dockerCommand="sudo -S docker exec -i zippy bash -c 'source /home/zippy/zippy_ws/install/setup.bash && echo \"\" && echo \"Topic SAVIOUR_ERROR of Robot no. ${id}\" && rostopic echo /saviour_error'"
        sshpass -p ${robotpassword} ssh -t -oStrictHostKeyChecking=no zippy@${robotIP} "echo ${robotpassword} | ${dockerCommand}"

    #Display /barcode_pose_raw topic using (-BP)
    elif [[ "$setTarget" == "-BP" ]]; then
        echo "=== Entering Docker container on Robot ${id} (${robotIP}) and reading /barcode_pose_raw ==="
        dockerCommand="sudo -S docker exec -i zippy bash -c 'source /home/zippy/zippy_ws/install/setup.bash && echo \"\" && echo \"Topic BARCODE_POSE_RAW of Robot no. ${id}\" && rostopic echo /barcode_pose_raw'"
        sshpass -p ${robotpassword} ssh -t -oStrictHostKeyChecking=no zippy@${robotIP} "echo ${robotpassword} | ${dockerCommand}"

    #SCP saviour files using (-S)
    elif [[ "$setTarget" == "-S" ]]; then
        command1="echo ${robotpassword} | sudo -S docker rm -f zippy robot_configurator_backend robot_configurator_frontend"
        command2="echo ${robotpassword} | sudo -S rm -rf saviour"
        command3="echo ${robotpassword} | sudo -S unzip saviour.zip && echo ${robotpassword} | sudo -S rm -rf saviour.zip"
        sshpass -p ${robotpassword} ssh -oStrictHostKeyChecking=no zippy@${robotIP} ${command1}
        sshpass -p ${robotpassword} ssh -oStrictHostKeyChecking=no zippy@${robotIP} ${command2}
        echo "sshpass -p ${robotpassword} scp -r ${saviourpath} zippy@${robotIP}:/home/zippy/"
        sshpass -p ${robotpassword} scp -r ${saviourpath} zippy@${robotIP}:/home/zippy/
        sshpass -p ${robotpassword} ssh -oStrictHostKeyChecking=no zippy@${robotIP} ${command3}

    #Display Error logs from robot using (-SH)
    elif [[ "$setTarget" == "-SH" ]]; then
        echo "=== Reading error logs on Robot ${id} (${robotIP}) ==="
        command="cd /home/zippy/robot_parameters/roslogger_bags/TRACE && cat error.log | grep -a Error"
        sshpass -p ${robotpassword} ssh -o StrictHostKeyChecking=no zippy@${robotIP} "${command}"

    #Display /robot_control/cmd_vel topic using (-CR)
    elif [[ "$setTarget" == "-CR" ]]; then
        echo "=== Entering Docker container on Robot ${id} (${robotIP}) and reading /robot_control/cmd_vel ==="
        dockerCommand="sudo -S docker exec -i zippy bash -c 'source /home/zippy/zippy_ws/install/setup.bash && echo \"\" && echo \"Topic ROBOT_CMD_VALUE of Robot no. ${id}\" && rostopic echo /robot_control/cmd_vel'"
        sshpass -p ${robotpassword} ssh -t -oStrictHostKeyChecking=no zippy@${robotIP} "echo ${robotpassword} | ${dockerCommand}"

    fi
done
