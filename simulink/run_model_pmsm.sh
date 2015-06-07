#!/bin/sh

if [ -n "$2" ] ; then

  if [ -e target-ssh-key ] ; then
    SSH_LOCAL_KEY_OPT="-i target-ssh-key"
  fi

  xterm -e "scp $SSH_LOCAL_KEY_OPT ./$1 root@$2:/tmp ; echo model $1 copied to target is run ; ssh $SSH_LOCAL_KEY_OPT root@$2 /tmp/$1 -tf inf -w ; sleep 2" &

else

  xterm -e "./$1 -tf inf -w ; sleep 2" &

fi
