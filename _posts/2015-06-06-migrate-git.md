---
layout: post
title: "How to migrate a git repository"
tags:
- Git
---


## How to migrate

1. Pull the latest changes from the current origin

```
git fetch 
git fetch --tags
git pull --all
```

2. Checkout all the remote branches locally (note this is a local operation does not require connection to the origin)

```
for remote in `git branch -r| grep -v HEAD`; do git checkout --track $remote ; done
```

3. Create new remote (migration target). This repository has to be created on your service of choice.

```
git remote add new-origin <url-of-new-origin>
```

4. Push all checked out branches and tags to the new origin

```
git push --all
```

5. Push all previosly feteched tags to the new remote

```
git push --tags
```
