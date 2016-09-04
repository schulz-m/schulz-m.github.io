---
layout: post
comments: true
title: "Subscribe and Access Topic in Rospy regardless of Message Type"
tags:
- Rospy
- ROS
---

## Introduction

A common problem I encountered a lot of times is that you simply want to subscribe to a topic and access a certain field of the published message without knowing its type. It turns out that this is rather simple to do in python by using the [Connection Header](http://wiki.ros.org/ROS/Connection%20Header) of the handy [AnyMsg](http://docs.ros.org/jade/api/rospy/html/rospy.msg.AnyMsg-class.html) that lets you subscribe to any topic regardless of type.

<!--more-->

## Example Class

See the following simple example class that subscribes to `some_topic` and prints its field `known_field`, nicely using python's idiom of [duck typing](https://en.wikipedia.org/wiki/Duck_typing).

```python
import rospy
import sys

from importlib import import_module

class Listener(object):
    def __init__(self):
        self._binary_sub = rospy.Subscriber(
            'some_topic', rospy.AnyMsg, self.binary_callback)

    def binary_callback(self, data):
        assert sys.version_info >= (2,7) #import_module's syntax needs 2.7
        connection_header =  data._connection_header['type'].split('/')
        ros_pkg = connection_header[0] + '.msg'
        msg_type = connection_header[1]
        print 'Message type detected as ' + msg_type
        msg_class = getattr(import_module(ros_pkg), msg_type)
        self._binary_sub.unregister()
        self._deserialized_sub = rospy.Subscriber(
            'some_topic', msg_class, self.deserialized_callback)

    def deserialized_callback(self, data):
        print data.known_field
```

## Explanation

As shortly mentioned, the ROS msg type `AnyMsg` lets you subscribe to any type of Msg, so the `__init__` function is rather straightforward, the problem with the data parsed to the callback is that the data is not deserialized in any way.

Inside the `binary_callback` we parse the connection header of the msg and assume that the type is defined inside the `msg` namespace of the given rospackage. To parse the header as an import, we use the [import_module](https://docs.python.org/3/library/importlib.html#importlib.import_module) function, such that we don't have to import all possible msg types we could encounter. This method is the reason to do a version check in the beginning.

After we imported the module and got the type of the incoming msg, we simply unregister the binary subscriber and create a subscriber with the known type. Note that this implementation only allows the class to listen to the first incoming MsgType on this topic.

