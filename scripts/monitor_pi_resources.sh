#!/bin/bash
echo "=== Pi Resource Monitor ==="
while true; do
    clear
    echo "Temperature: $(vcgencmd measure_temp)"
    echo "CPU: $(top -bn1 | grep "Cpu(s)" | awk '{print $2}')"
    echo "Memory: $(free -h | awk 'NR==2{printf "%.1f%%", $3*100/$2}')"
    echo "Docker Stats:"
    docker stats --no-stream --format "table {{.Name}}\t{{.CPUPerc}}\t{{.MemUsage}}"
    sleep 2
done