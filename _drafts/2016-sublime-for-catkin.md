---
layout: post
title: "Sublime with Catkin"
tags:
- Sublime
- Cpp
- Catkin
- ROS
---

## Introduction

This post is about the usage of sublime with the catkin infrastructure used by ROS. This was tested with **Ubuntu 14.04 LTS** and **ROS Indigo**. The post assumes that you are familiar with both **ROS** and **catkin** and that you have a catkin workspace working with the usual folder structure in `~/catkin_ws`. Another nice introduction into using sublime as your IDE can be found at [Linux Sublime Dev](https://chromium.googlesource.com/chromium/src/+/master/docs/linux_sublime_dev.md), this post aims to provide a simple overview and setup for **catkin** specifically.

## Sublime Installation

For those of you who have never heard of sublime, here a quick and dirty installation guide to have the setup that I am working with. First install sublime 3:

```raw
sudo add-apt-repository ppa:webupd8team/sublime-text-3
sudo apt-get update
sudo apt-get install sublime-text-installer
```

Everything gets installed for you, such that you can open sublime with a simple `subl` command in the terminal. For all real cool features of the program you need to install the so called [package_control](https://packagecontrol.io/installation).

Some useful starting points:

* For some of the common commands, consult the [cheatsheet](http://csnipp.com/s/66/-List-of-commonly-used-sublime-text-commands)
* In general, to edit your preferences (such as tabbing), simply open `Preferences -> Settings - User` and overwrite settings you find in `Preferences -> Settings - Default`
* If you are not satisfied with the syntax detected by sublime you can simply change it with `View -> Syntax -> Open all with current extension as... ->[your syntax choice]`

Some other neat extensions:

* [Markup editing with sublime](http://plaintext-productivity.net/2-04-how-to-set-up-sublime-text-for-markdown-editing.html)
* [Sublime Linter for Code Style](http://www.sublimelinter.com/en/latest/)
* [Increment Selection](https://github.com/yulanggong/IncrementSelection)

Some of the useful keybindings to show the power of sublime:

* TEXT EDITING
* `ctrl + d` multi-select occurances of pattern
* `ctrl + click` multi-cursor selection
* WINDOW NAVIGATION
* `shift + alt + num` Number of split windows in editor
* FILE NAVIGATION
* `ctrl + p` search box to look for files inside the opened folder
* `ctrl + o` to use the OS navigation to open another file in sublime
* `alt + o` to switch between source and header files (only works for projects)

## Set up the catkin project

Open sublime and then simply add your `catkin_ws/src` folder by using `Project -> Add Folder to Project`, then save it as a project file in a location of your joice by `Project -> Save Project as...`.

Now when you open sublime, by default you should have your catkin workspace in the file navigator.

As a next step, you want to use CTags to jump through definitions in your source code. The source of the CTags Plugin is found [here](https://github.com/SublimeText/CTags). You can simply install it by using the package manager. As a refresher: Type `Alt + Shift + P` and type install. Type `ctags` and put enter. For ctags to work properly, ctags also has to be installed on the OS via `sudo apt-get install exuberant-ctags`.

To build the indices just right click on your project folder and click `CTags: Rebuild Tags` in the context menu.

--------- DEPRECATED
Install sublime-clang (Warning: Apparently deprecated soon) [link](https://chromium.googlesource.com/chromium/src/+/master/docs/linux_sublime_dev.md#Code-Completion-with-SublimeClang-Linux-Only)

Show how to install [SublimeRosAssist](https://github.com/groundmelon/SublimeRosAssist)

```
cd ~/.config/sublime-text-3/Packages
git clone https://github.com/groundmelon/SublimeRosAssist.git

```

At some point it would be nice to combine the featured CTags to work with this.

[ycmd backend](https://github.com/Valloric/ycmd)
[cpp youcompleteme](https://github.com/glymehrvrd/CppYCM)
[sublime ros assist](https://github.com/groundmelon/SublimeRosAssist)
[script to create project / doesn't work at all](https://gist.github.com/wjwwood/5273972)

## Setup specific for ROS projects

There are several nice features offered specifically for project development in ROS.

### ROS Msg File Syntax

Download the syntax file from [Github](https://gist.github.com/eric-wieser/1cd2919483d18d6b3788a54dec4f165c) and copy the file into `~.config/sublime-text-3/Packages/User`. Now you can simply select *ROS message definition* with `View -> Syntax -> Open all with current extension as...`.

...
