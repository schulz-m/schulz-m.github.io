# Incremental backups with rsync

The search for this script came from the need of making incremental backups to be as transparent as possible. This way only a file system explorer is necessary to be able to browse and access any version of the backup.

The transparency of this backup strategy comes from the fact that there is no propietary formats or archives involved everythig is saved in the filesystem with the aid of hardlinks which save a lot of space instead of multiple copies of the same files. A very detailed explanation of how this works can be found in [this](http://www.admin-magazine.com/Articles/Using-rsync-for-Backups) article.

The following script is a slightly modified version of the one proposed in the above linked article. The modificaitons make it a bit easier to configure and adds an exclude file to exclude files and folders from the backup.

The newest backup is "backup.0" and the others are the older versions of the backup. With a bit of tinkering, i.e. some scripting and with the use of cron jobs, this scheme can be extended to take hourly, dailiy weekly and monthly backups all linked with each other. This might be topic of a future entry on this page.

```bash
#!/bin/bash

echo "\nIncremental rsync backup scirpt"
echo   "==============================="

# Source and destination of the backup 
# ABSOLUTE PATHS!
SOURCE_DIR=/media/data
BACKUP_DIR=/media/backup-green1

# Get exclude file path
# This file has to be in the same directory of the shell script
echo "Getting exclude list for backup:"
DIR="`dirname \"$0\"`"          # relative
DIR="`( cd \"$DIR\" && pwd )`"  # absolutized and normalized
if [ -z "$DIR" ] ; then
  # error; for some reason, the path is not accessible
  # to the script (e.g. permissions re-evaled after suid)
  exit 1  # fail
fi
EXCLUDE_FILE="${DIR}/exclude-list.txt"
echo $EXCLUDE_FILE
echo "Excluding following from backup:"
cat $EXCLUDE_FILE
echo ""

echo "Switching working directory to bakcup location:"
cd $BACKUP_DIR
pwd

rm -rf backup.3
mv backup.2 backup.3
mv backup.1 backup.2
mv backup.0 backup.1
rsync -ah --info=progress2 --delete --exclude-from=$EXCLUDE_FILE --link-dest= backup.1/ $SOURCE_DIR backup.0/
```

# Sources
http://www.admin-magazine.com/Articles/Using-rsync-for-Backups


[gimmick:Disqus](psimona-github-io)
