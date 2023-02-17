#!/bin/bash

# usr_name_1=newslab
# host_ip_1=140.112.28.112
# port_1=22222
src_path_1="/home/newslab/wayside-portal/recording/*/"
dest_ip=140.112.28.112
dest_path_1="/home/newslab/recording"
# mkdir -p ${dest_path_1}
# addr_1=${usr_name_1}@${host_ip_1}

# usr_name_2=newslab
# host_ip_2=140.112.28.112
# port_2=22223
# src_path_2=/home/newslab/wayside-portal/recording/
# dest_path_2=$(pwd)/recording_2/
# mkdir -p ${dest_path_2}
# addr_2=${usr_name_2}@${host_ip_2}


# rsync -avzh -e "ssh -p ${port_1}" --progress ${addr_1}:${src_path_1} ${dest_path_1}
# rsync -avzh -e "ssh -p ${port_2}" --progress ${addr_2}:${src_path_2} ${dest_path_2}
rsync -aPh ${src_path_1} ${dest_ip}:${dest_path_1}