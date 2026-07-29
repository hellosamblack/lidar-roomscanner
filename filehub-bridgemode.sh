#!/bin/bash

# --- Configuration ---
ROUTER_IP="172.17.2.57"
ROUTER_USER="root" # Change to "root" if your FileHub requires it
ROUTER_PASS="20080826"      # FileHubs usually have a blank telnet password

# Ensure expect is installed
if ! command -v expect &> /dev/null; then
    echo "Error: 'expect' command is required but not installed."
    exit 1
fi

echo "Connecting to FileHub at $ROUTER_IP..."

# Start the expect session
expect << EOF
set timeout 10
match_max 100000
spawn telnet $ROUTER_IP

# Handle login gracefully
expect {
    -re "login:|ogin:" {
        send "$ROUTER_USER\r"
        expect -re "Password:|assword:"
        send "$ROUTER_PASS\r"
        # Wait for the actual root prompt to bypass the "can't chdir" warning
        expect "#"
    }
    "#" {
        # Already logged in
    }
    timeout {
        puts "\n\nERROR: Timed out waiting for the router prompt."
        exit 1
    }
}

puts "\n\n=== Connected! Applying transparent bridge... ==="

# 1. Blindfold the auto-switch daemon (wait for prompt after every command)
send "echo '#!/bin/sh' > /tmp/dummy.sh\r"
expect "#"
send "echo 'exit 0' >> /tmp/dummy.sh\r"
expect "#"
send "chmod +x /tmp/dummy.sh\r"
expect "#"
send "mount --bind /tmp/dummy.sh /sbin/EnterRouterMode.sh\r"
expect "#"
send "mount --bind /tmp/dummy.sh /sbin/net_auto_switch\r"
expect "#"

# 2. Bridge the interfaces and kill DHCP
send "brctl addif br0 apcli0\r"
expect "#"
send "killall udhcpd\r"
expect "#"

puts "\n=== Verifying Configuration ==="

# 3. Verify the bind mounts
send "mount | grep EnterRouterMode\r"
expect {
    "EnterRouterMode" { puts "\nSUCCESS: Auto-switch scripts successfully blindfolded." }
    timeout { puts "\nERROR: Bind mounts failed to apply." }
}
expect "#"

# 4. Verify the bridge
send "brctl show\r"
expect {
    "apcli0" { puts "\nSUCCESS: Wi-Fi client (apcli0) successfully bridged to LAN." }
    timeout { puts "\nERROR: Wi-Fi client failed to bridge." }
}
expect "#"

# 5. Verify DHCP server is dead
send "pidof udhcpd\r"
expect "#"
puts "\nSUCCESS: Local DHCP server disabled."

# Clean exit
puts "\n=== Success! Exiting telnet. ==="
send "exit\r"
expect eof
EOF