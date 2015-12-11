#!/bin/bash
function _func_rosserver() {
	if [ -z "$eth0_addr" ]; then
		echo "No eth0 connection..."
		if [ -z  "$wlan0_addr" ]; then
			echo "No wlan0 connection..."
			export ROS_MASTER_URI=http://localhost:11311
			unset ROS_HOST_NAME
			unset ROS_IP
			echo ""
			echo "            !!CAUTION!! "
			echo "  There're no Ethernet connection."
			echo "  We CAN NOT set this PC to ROS server."
			echo ""
		else
			echo "There are wlan0 connection"
			export ROS_MASTER_URI=http://${wlan0_addr}:11311
			export ROS_HOST_NAME=${wlan0_addr}
			export ROS_IP=${wlan0_addr}
			export PS1="\[\033[41;1;33m\]<ROS_server>\[\033[0m\]\w$ "
		fi	
	else
		echo "There are eth0 connection"
		export ROS_MASTER_URI=http://${eth0_addr}:11311
		export ROS_HOST_NAME=${eth0_addr}
		export ROS_IP=${eth0_addr}
		export PS1="\[\033[41;1;33m\]<ROS_server>\[\033[0m\]\w$ "
	fi

	env | grep "ROS_MASTER_URI"
	env | grep "ROS_HOST_NAME"
	env | grep "ROS_IP"
}

function _func_rosclient() {
	if [ -z "$1" ]; then
		echo $1
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
				echo "  There're no Ethernet connection."
				echo "  We CAN NOT set this PC to ROS client."
				echo ""
			else
				echo "There are wlan0 connection"
				export ROS_MASTER_URI=http://$1:11311
				export ROS_HOST_NAME=${wlan0_addr}
				export ROS_IP=${wlan0_addr}
				export PS1="\[\033[44;1;33m\]<ROS_client>\[\033[0m\]\w$ "
			fi	
		else
			echo "There are eth0 connection"
			export ROS_MASTER_URI=http://$1:11311
			export ROS_HOST_NAME=${eth0_addr}
			export ROS_IP=${eth0_addr}
			export PS1="\[\033[44;1;33m\]<ROS_client>\[\033[0m\]\w$ "
		fi
		env | grep "ROS_MASTER_URI"
		env | grep "ROS_HOST_NAME"
		env | grep "ROS_IP"
	fi
}

function _func_roslocal() {
	export ROS_MASTER_URI=http://localhost:11311
	unset ROS_HOST_NAME
	unset ROS_IP
	export PS1="\[\033[42;1;33m\]<ROS_local>\[\033[0m\]\w$ "
	env | grep "ROS_MASTER_URI"
	env | grep "ROS_HOST_NAME"
	env | grep "ROS_IP"
}

function _func_rosexit(){
	export ROS_MASTER_URI=http://localhost:11311
	unset ROS_HOST_NAME
	unset ROS_IP
	export PS1="\u@\h:\w\\$ "
}

function _func_comp_rosaddress(){
	local cur=${COMP_WORDS[COMP_CWORD]}
	if [ "$COMP_CWORD" -eq 1 ]; then
		COMPREPLY=( $(compgen -W "server client local exit" -- $cur) )
	fi
}

function _func_rosaddress() {
	# Get now eth0 or wlan0 IP address
	eth0_addr=$(ip -f inet -o addr show eth0|cut -d\  -f 7 | cut -d/ -f 1)
	wlan0_addr=$(ip -f inet -o addr show wlan0|cut -d\  -f 7 | cut -d/ -f 1)

	if [ $1 = "local" ]; then
		_func_roslocal
	elif [ $1 = "server" ]; then
		_func_rosserver
	elif [ $1 = "client" ]; then
		_func_rosclient $2
	elif [ $1 = "exit" ]; then
		_func_rosexit
	fi
	
}

alias rosaddress=_func_rosaddress
complete -o default -F _func_comp_rosaddress rosaddress
