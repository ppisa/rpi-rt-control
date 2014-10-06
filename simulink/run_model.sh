#!/bin/sh

xterm -e "scp -i target-ssh-key ./$1 root@$2:/tmp ; echo model $1 copied to target is run ; ssh  -i target-ssh-key root@$2 /tmp/$1 -tf inf -w ; sleep 2" &
