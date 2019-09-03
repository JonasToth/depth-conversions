# Git Commands

This document describes all relevant `git` commands for this repository.

## subtree

The repository contains 3rd-party libraries inside the repository, using the
`git subtree` command.
This scheme is called **monorepo** and allows to have all depencencies under
control. It gives better ways for updating and versioning the project.

### A note on patching dependencies

Patching the dependencies needs to be done in commits that just touch a
subproject. Please see proper documentation in the internet for reference.

### Adding a dependency

- first add the project as a new remote (please follow the naming scheme
  `upstream-<projectname>`

```bash
$ git remote add -f upstream-<projectname> git@github.com/projectname/repo.git
> Fetching the upstream repository through '-f'
```

- then do a `subtree` merge into the repository. Note that all projects are
  flat in the repository, so the `--prefix` should be
  `$GIT_ROOT/<projectname>`
- use a proper release and don't build directly from master if possible.

```bash
$ git subtree add --prefix third_party/<projectname> upstream-<projectname> <REVISION> --squash
> Squashes all source into the directory '<projectname>' from the '<REVISION>'
> The commit-id (<REVISION>) should correspond to a proper release.
```

[References](https://www.atlassian.com/blog/git/alternatives-to-git-submodule-git-subtree)

### Updating a dependency

- `fetch` the dependency to get the new code

```bash
$ git fetch upstream-<projectname>
> Retrieving the changes
```

- Then pull the changes into the appropriate directory

```
$ git subtree pull --prefix third_party/<projectname> upstream-<projectname> <REVISION> --squash
> Merging the changes into the project directory
```

Please note that the `<REVISION>` should correspond to a proper release intead
of `master` or similar.

### Keeping the repo clean

Use `git gc` to garbage-collect commits fetched from 3rd-party repositories
that are not necessary to exist.
