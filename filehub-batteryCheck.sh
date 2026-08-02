# --- Configuration ---
ROUTER_IP="172.17.2.57"
ROUTER_USER="root" # Change to "root" if your FileHub requires it
ROUTER_PASS="20080826"      # FileHubs usually have a blank telnet password

expect -c '
spawn telnet 172.17.2.57
expect "login:" { send "root\r" }
expect "Password:" { send "20080826\r" }
expect "#" { send "cat /proc/vs_battery_quantity\r" }
expect "#" { send "exit\r" }
'