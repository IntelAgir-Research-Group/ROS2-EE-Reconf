#!/bin/bash

# https://github.com/kentcz/rapl-tools

date_file_name=`date +"%Y-%m-%d_%H:%M:%S"`
echo "timestamp,date_time,total_energy" > /data/$date_file_name-energy.csv

while true; do

    # Run PowerTOP and save the output to a temporary file
    powertop -t 1 --csv > powertop_output.csv
    
    sum_energy=0.0
    for l in `cat powertop.csv | grep component_container_isolated | awk '{ print $3 }'`
    do
    	word=`tr -s ';' ' ' <<< "$l"`
    	energy=`tr -s ',' '.' <<< "$word" | awk '{ print $1 }'`
    	sum_energy=`echo $sum_energy + $energy | bc`
    	echo $energy
    done
    
    timestamp=`date +%s`
    date_time=`date +"%Y-%m-%d %H:%M:%S"`
    
    echo "$timestamp, $date_time, $sum_energy" >> /data/$date_file_name-energy.csv
    
    #echo "Sum: $sum_energy"
done
