---
layout: post
title: "Tmux"
tags:
- Tmux
---

Tmux is a program used to multiplex several virtual consoles akin to GNU Screen (parphrased form [Wikipedia](https://en.wikipedia.org/wiki/Tmux)). This means that whith this program one can open a new shell and exit it while this shell keeps running or also one can reconnect to an alredy running shell.

This is very practical when connect over SSH to a remote computer as one might not want to interrupt a task if suddently the connection to the remote host is lost. Also imagine running an intensive task on a remote machine and then having to terminate the connection because you have to shutdown your computer or similar.

Below there is a biref cheat sheet which was obtained by seraching for [tmux cheat sheet on Duck Duck go](https://duckduckgo.com/?q=tmux+cheat+sheet&ia=answer).

## Session Control (from the command line)
| Command        | Note                                                        |
|:---------------|:------------------------------------------------------------|
| tmux 	         | Start a new session                                         |
| tmux attach    | Re-attach a detached session                                |
| tmux attach -d | Re-attach a detached session (and detach it from elsewhere) |

## Pane Control (from whithin tmux)
| Command        | Note                                                        |
|:---------------|:------------------------------------------------------------|
| Ctrl-B "       | Split pane horizontally                                     |
| Ctrl-B %       | Split pane vertically                                       |
| Ctrl-B o       | Next pane                                                   |
| Ctrl-B ;       | Previous pane                                               |
| Ctrl-B x       | Kill current pane                                           |
| Ctrl-B !       | Kill all panes but the current one                          |
| Ctrl-B Ctrl-O  | Swap panes                                                  |
| Ctrl-B t       | Display clock                                               |
| Ctrl-B q       | Transpose two letters (delete and paste)                    |

## Window Control (from whithin tmux)
| Command        | Note                                                        |
|:---------------|:------------------------------------------------------------|
| Ctrl-B c       | Create new window                                           |
| Ctrl-B d       | Detach from session                                         |
| Ctrl-B ,       | Rename a window                                             |
| Ctrl-B w       | List windows                                                |

## Copy-Mode (Emacs) (from whithin tmux)
| Command        | Note                                                        |
|:---------------|:------------------------------------------------------------|
| Ctrl-B [       | Enter copy mode |
| Ctrl-B M-<     | Bottom of history |
| Ctrl-B M->     | Top of histroy |
| Ctrl-B M-w     | Copy selection |
| Ctrl-B M-y     | Paste selection |
| Ctrl-B Up      | Cursor Up |
| Ctrl-B Down    | Cursor Down |
| Ctrl-B Left    | Cursor Left |
| Ctrl-B Right   | Cursor Right |

## Copy-Mode (vi) (from whithin tmux)
| Command        | Note                                                        |
|:---------------|:------------------------------------------------------------|
| Ctrl-B [       | Enter copy mode |
| Ctrl-B G       | Bottom of history |
| Ctrl-B g       | Top of histroy |
| Ctrl-B Enter   | Copy selection |
| Ctrl-B p       | Paste selection |
| Ctrl-B k       | Cursor Up |
| Ctrl-B j       | Cursor Down |
| Ctrl-B h       | Cursor Left |
| Ctrl-B l       | Cursor Right |

