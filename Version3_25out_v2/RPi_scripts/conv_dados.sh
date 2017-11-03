#!bin/bash

#set -e

while read -r -a myArray #< <(stdbuf -o L gatttool -b $1 -t random --char-write-req -a $2 -n 0100 --listen | stdbuf -o L sed -e "s/Notification handle = $3 value: //" -e '/Characteristic/d')

do 
  pkill gatttool
  echo ${myArray[*]}
  exit
done < <(stdbuf -o L gatttool -b $1 -t random --char-write-req -a $2 -n 0100 --listen | stdbuf -o L sed -e "s/Notification handle = $3 value: //" -e '/Characteristic/d')
