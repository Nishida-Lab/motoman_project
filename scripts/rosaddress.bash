#!/bin/bash
rosserver() {
	# eth0とwlan0の現在のIPアドレスを取得
	eth0_addr=$(ip -f inet -o addr show eth0|cut -d\  -f 7 | cut -d/ -f 1)
	wlan0_addr=$(ip -f inet -o addr show wlan0|cut -d\  -f 7 | cut -d/ -f 1)

	if [ -z "$eth0_addr" ]; then
		echo "No eth0 connection..."
		if [ -z  "$wlan0_addr" ]; then
			echo "No wlan0 connection..."
			export ROS_MASTER_URI=http://localhost:11311
			unset ROS_HOST_NAME
			unset ROS_IP
			echo ""
			echo "            !!CAUTION!! "
			echo "  There're no Internet connection."
			echo "  We CAN NOT set this PC to ROS server."
			echo ""
		else
			echo "There are wlan0 connection"
			export ROS_MASTER_URI=http://${wlan0_addr}:11311
			export ROS_HOST_NAME=${wlan0_addr}
			export ROS_IP=${wlan0_addr}
			export PS1="\[\033[44;1;37m\]<ROS_server>\[\033[0m\]\w$ "
		fi	
	else
		echo "There are eth0 connection"
		export ROS_MASTER_URI=http://${eth0_addr}:11311
		export ROS_HOST_NAME=${eth0_addr}
		export ROS_IP=${eth0_addr}
		export PS1="\[\033[44;1;37m\]<ROS_server>\[\033[0m\]\w$ "
	fi

	env | grep "ROS_"
}

rosclient() {
	# eth0とwlan0の現在のIPアドレスを取得
	eth0_addr=$(ip -f inet -o addr show eth0|cut -d\  -f 7 | cut -d/ -f 1)
	wlan0_addr=$(ip -f inet -o addr show wlan0|cut -d\  -f 7 | cut -d/ -f 1)

	if [ -z "$1" ]; then
		echo "Input the ROS server's IP address.'"
	else
		if [ -z "$eth0_addr" ]; then
			echo "No eth0 connection..."
			if [ -z  "$wlan0_addr" ]; then
				echo "No wlan0 connection..."
				export ROS_MASTER_URI=http://localhost:11311
				unset ROS_HOST_NAME
				unset ROS_IP
				echo ""
				echo "            !!CAUTION!! "
				echo "  There're no Internet connection."
				echo "  We CAN NOT set this PC to ROS client."
				echo ""
			else
				echo "There are wlan0 connection"
				export ROS_MASTER_URI=http://$1:11311
				export ROS_HOST_NAME=${wlan0_addr}
				export ROS_IP=${wlan0_addr}
				export PS1="\[\033[42;1;37m\]<ROS_client>\[\033[0m\]\w$ "
			fi	
		else
			echo "There are eth0 connection"
			export ROS_MASTER_URI=http://$1:11311
			export ROS_HOST_NAME=${eth0_addr}
			export ROS_IP=${eth0_addr}
			export PS1="\[\033[44;1;37m\]<ROS_client>\[\033[0m\]\w$ "
		fi
		env | grep "ROS_"
	fi

}

roslocal() {
	export ROS_MASTER_URI=http://localhost:11311
	unset ROS_HOST_NAME
	unset ROS_IP
	env | grep "ROS_"
}

alias rosserver=rosserver
alias rosclient=rosclient
alias roslocal=roslocal
