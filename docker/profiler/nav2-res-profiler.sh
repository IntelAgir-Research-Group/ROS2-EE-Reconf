#!/bin/bash

echo "Profiling the resource usage of Nav2 container"

nav2_pid=`ps ax | grep nav2_container | head -1 |  awk '{ print $1}'`

date_file_name=`date +"%Y-%m-%d_%H:%M:%S"`

echo "timestamp,date_time,cpu_pct,mem_pct,total_mem" > /data/$date_file_name-resource.csv

while true
do
    timestamp=`date +%s`
    date_time=`date +"%Y-%m-%d %H:%M:%S"`
    cpu_pct=`pidstat -u -p $nav2_pid | tail -1 | awk '{ print $8 }'`
    mem_pct=`pidstat -r -p $nav2_pid | tail -1 | awk '{ print $8 }'`
    total_mem=`pmap $nav2_pid | tail -1 | awk '{ print $2}'`

    echo "$timestamp, $date_time, $cpu_pct, $mem_pct, $total_mem" >> /data/$date_file_name-resource.csv
done
