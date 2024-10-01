# Start Jenkins

1.  ```cd ~/webpage_ws && bash start_jenkins.sh```

2.  ```cat ~/webpage_ws/jenkins__pid__url.txt```
3.  Copy Jenkins URL to browser

4.  Log into Jenkins
    Username: ```tester```
    Password: ```tester123```

    Now you are inside Jenkins!

# Push code to this repository

1.  ```cd ~/ros2_ws/src/ros2_ci```
2.  Make any changes to the repo
3.  ```git commit -m"Test #n"``` where n = number of test. You can do a modification before commiting
4.  ```git push```

So see a new build inside Jenkins!
