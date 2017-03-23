---
layout: post
comments: true
title: "Python Threading with External Processes"
tags:
- Python
- Multithreading
---

# Introduction

Often when you write python scripts you might realize that the steps in one of your loops are actually not dependent on each other, so you could [parallelize](https://en.wikipedia.org/wiki/Parallel_computing) them. In it's simplest form, to parallelize computations python offers ready made *threads* and *locks*. There are two distinct ways to parallelize computation in python, either through Multiprocessing (and the according multiprocessing package) or Threading (and the according threading package), the pros and cons are excellently summarized by Jeremy Brown on [stackoverflow](http://stackoverflow.com/a/3046201/5882522). This post only covers a toy example for the use of the threading package.

<!--more-->

# Threading package

The threading package offers the usage of the threads of the underlying OS to be able to execute some job while already starting other jobs. Note that the package doesn't circumvent the so called [Global Interpreter Lock](https://docs.python.org/2/glossary.html#term-global-interpreter-lock) that prevents to have python threads running truly parallel on the same interpreter, i.e. if you run your python script the interpreter will always lock all the resources. That's why in this post we will only focus on running an *external process* in parallel as for example a `cmake` command.

# Parallel CMake Generator

The following example is a script that checks your current directories for `CMake` projects by checking if their is a `CMakeLists.txt`, it then creates a folder on a `build` folder and executes the corresponding `cmake` command. It does that while trying to parallelize over all the cores that the operating system provides. The code is structured in a way that every `cmake` to invoke is handled by a `Job` that is inserted in a `Queue` that get's done by `Workers`

```python
#!/usr/bin/python

import os
import Queue
import commands
from multiprocessing import cpu_count
from threading import Thread, Lock

class Worker(Thread):
    def __init__(self, queue):
        self.queue = queue
        super(Worker, self).__init__()

    def run(self):
        # Race condition, just try!
        while True:
            try:
                job = self.queue.get_nowait()
                job.execute()
                self.queue.task_done()
            except Queue.Empty:
                return

class Job(object):
    def __init__(self, app_name, stdout_mutex):
        self.app_name = app_name
        self.stdout_mutex = stdout_mutex

    def execute(self):
        # Actual Work
        cmake_cmd = commands.getoutput('cmake -Bbuild/{} -H{} -DCMAKE_BUILD_TYPE=Debug' \
            .format(self.app_name, self.app_name))

        with self.stdout_mutex:
            # Output printing
            print '----- GENERATED CMAKE for {}-----'.format(self.app_name)

            # Check if cmake was succesful or not
            if 'CMake Error' in cmake_cmd:
                print '==> Failed'
            else:
                print '==> Success'

if __name__ == "__main__":
    # Parse desired processors
    cpu_used = cpu_count()

    # Define stdout_mutex and fill up queue of jobs
    stdout_mutex = Lock()
    cmake_queue = Queue.Queue()

    for app_name in filter(os.path.isdir, os.listdir(os.getcwd())):
        if os.path.exists(os.path.join(app_name, 'CMakeLists.txt')):
            # Create directories and fill job queues
            os.makedirs('build/' + app_name)
            cmake_queue.put(Job(app_name, stdout_mutex))

    # Work on cmake
    for _ in range(cpu_used):
        Worker(cmake_queue).start()
    cmake_queue.join()

    print '\n==> Finished generating Makefiles <==\n'
```

# Explanation

## Worker

The worker gets the job done, he is simply designed as a subclass of the [thread object](https://docs.python.org/2/library/threading.html#thread-objects) that provides the threading activity in the `start` method, that itself will use the `run` method which is the method you want to override. We get the common job queue in the constructor and in the `run` method we acquire the job, execute it and tell it that the job is done. Note that there is a potential [race condition](https://en.wikipedia.org/wiki/Race_condition) if you want to check if the queue is empty that is while you are checking if the queue is empty, another worker might already take the last job in the queue, so the best way is to simply pass the appropriate exception.

## Job

The job object is simply to package the code to execute as a job in a queue. In the execution, we use the `commands` package to invoke the `cmake` command on the specific application and afterwards acquire a mutex for uninterrupted printing of the result.

## Main code

The code is pretty much self explanatory with the introduction, there are just two details: For one, the `os.makedirs` will fail if the directory already exist, which can easily be circumvented by these [proposed methods](http://stackoverflow.com/questions/600268/mkdir-p-functionality-in-python). The second detail is that we use the queues `join` method to determine if all the jobs have been done, this method is blocking until the target is achieved.