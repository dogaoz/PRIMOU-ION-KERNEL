#!/bin/bash

##################################################
#                                                #
#         Simon Sickle Gerrit push script.       #
#        Pushes to http://simonsickle.com        #
#     Created by: kalaker and simonsimons34      #
#               License: GNU GPLv2               #
#                                                #
##################################################

clear #Clears the terminal window
project="android_kernel_htc_msm7x30"
branch=$1 #$2 = The second data in the command line after ./gerrit.sh
name="Simon Sickle Gerrit"
url="http://review.simonsickle.com/"
if [ -f ~/.ssh/ssg_username ] #Checks if username file exists
then #If it does then:
un=`cat ~/.ssh/ssg_username`  #Sets the username ($un) from the .ssh file
echo "Welcome back to $name, $un!"
echo "You've already been through setup already so let's continue!"
echo "Your username is ${un}. To change this, execute 'rm -rf ~/.ssh/tbr_username'."
else #If it doesn't exist, let's create it.
echo "Welcome to $name"
echo "You need to setup your account."
echo "If you don't have a $name account, please go to $url and create one."
echo -n "What is your $name username: "
read un #'$un' = Username
echo $un > ~/.ssh/ssg_username #Echos the username to a file we can access in the future
echo "Your username is ${un}. To change this, execute 'rm -rf ~/.ssh/tbr_username'."
echo "BE SURE YOUR SSH KEYS ARE MATCHED WITH GERRIT IN YOUR SETTINGS"
fi #End username check "if"

if [ -z "$1" ] #Checks if $2 is empty
then
echo "Branch wname not given. Next time, please use the format './gerrit <project> <branch>'"
read -p "Branch name (case-sensitive): " branch #Sets the variable $branch from what the user enters
fi
un=`cat ~/.ssh/ssg_username` #Sets the username ($un) from the .ssh file (in case this wasn't done earlier)
while true; do
read -p "Have you already committed your changes [Y/N]? " yn #Asks user if they've committed. We could do this, but asking the user is easier
case $yn in
[Yy]* ) echo "Good, let's continue"; break;;
[Nn]* ) echo "Please go and commit your changes ('git commit -a')."; exit;;
    * ) echo "Please answer Yes or No.";; #Requires "yes" or "no"
esac #End case
done
echo "Push information:"
echo "Your username: $un" #Displays username to user
echo "Project: $project" #Displays project user is committing to
echo "Branch: $branch" #Displays branch user is commmitting to
while true; do
read -p "Are you sure you wish to push to Gerrit [Y/N]? " yn #Asks user if they are sure they want to commit
case $yn in
[Yy]* ) git push -u "ssh://$un@review.simonsickle.com:29418/$project" "HEAD:refs/for/$branch"; break;; #Pushes to gerrit

[Nn]* ) echo "Goodbye."; exit;; #Exits if user doesn't want to push
    * ) echo "Please answer Yes or No.";; #Requires "yes" or "no"
esac #End case
done
exit

