# Contributing to Exudyn

Currently, contributing is only possible by directly contacting the authors (e.g. writing an Issue or Discussion where you mention your requested change)
You can also write an email to reply.exudyn@gmail.com

Due to programming workflows and the very limited ressources for code review, the current repository is not fully open. 

**When you report a problem, start with the output of**

```
python -m exudyn info
```

It prints the version, the location of the installed package, which compiled module is loaded,
the Python version and platform, and which optional packages are present. That is what most
answers need to start from, and it saves a round of questions.


However, it would be opened if sufficient requests exist.


If a change of yours is agreed and you write C++, two conventions are worth reading first:
[`docs/dev/CODING_STYLE.md`](docs/dev/CODING_STYLE.md) in general, and its §10 in particular —
how Exudyn reports an error, and which of the nine exception types a new check should raise.
