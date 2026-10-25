# Contributions and the version: proposed issues and proposed resolutions

The design of revision2026c step RG11.11 (#3083, maintainer 2026-10-24 and 2026-10-25). A development note, not
documentation: the problem, and the mechanism as built with the maintainer's decisions.

## 1. The problem

The micro version is the count of closed issues (`releases.json` gives the count each release began at), and every
issue carries its number, `dateResolved` and `resolvedInVersion`. A contributor who raises or resolves an issue on a
branch therefore produces numbers and versions that the maintainer produces at the same time with other content:

- two branches that each raise an issue both create `issues/open/3084.json`, with different contents;
- two branches that each resolve an issue both produce version 1.13.190, for different code;
- `meta.json`, `version.txt`, `versionCpp.cpp`, the README line, `CHANGELOG.md` and the tracker pages conflict on every
  merge, and a hand-merge of them is wrong by construction.

## 2. Built: proposed issues

- `exudev issue propose "title" "description" --type T --author NAME [--effort --priority --file --line]` writes
  `tools/issueTracker/issues/proposed/<YYYYMMDD-HHMMSS>-<author>-<title>.json`: checked as an issue is, but without a
  number, so no version changes; the name makes two proposals of two branches different files - no merge conflict.
- `exudev issue add-proposed` raises the proposals in the order of their names - the proposer as author, the date of
  the proposal in the description - and removes the files. `exudev issue raise` (and `RaiseIssue`) runs it first, so a
  proposal is never overtaken by a later issue of the maintainer.
- Test: `python/testing/test_issueTracker.py::testProposedIssuesAreRaisedFirst`.

## 3. Built: proposed resolutions and merging (maintainer 2026-10-25)

**Rule: only the maintainer's branch numbers and resolves.** A contribution carries what it would have written to the
tracker as proposals, and the merge turns them into issues and resolutions in the maintainer's sequence.

1. **Proposed resolutions.** `exudev issue propose-resolution ISSUE "release notes" --author NAME` writes
   `issues/proposed/resolutions/<date>-<author>-<issue>.json`, ISSUE the number of an open issue or the file name of a
   proposal of the same branch, which gets its number when it is raised. Nothing else of the tracker changes.
2. **Applying them.** After the merge, `exudev issue apply-proposed`: first `add-proposed`, then the resolutions in
   ascending issue number, each with the date of the merge (decided: otherwise the versions would not follow the main
   branch) and the contributor as resolvedAuthor - the proposer is the author of a raised and of a resolved issue, as
   the one who did the work (decided). Each bumps the micro version once. A resolution of an issue that is no longer
   open is reported and its file kept.
3. **A deliberate command** (decided): `apply-proposed` changes the version and is not run by another verb; while
   proposals wait, every resolve prints a note with the command.
4. **The check of a contribution.** `tools/checkIssues.py --contribution BASE` lists what the branch changed against
   BASE (`git diff --name-only BASE...HEAD`) and refuses the numbered issue files (`open/`, `closed/`, `archive/`,
   `meta.json`), `releases.json`, `versionName.txt`, the version files, `CHANGELOG.md` and `trackerlog.md`; the proposals
   pass. The workflow `.github/workflows/contribution.yml` runs it on every pull request from a fork, against the base
   branch (#3084, maintainer 2026-10-25); the pull requests of the maintainer's own branches are not checked.
5. **Conflicts in generated files** are never merged by hand: the maintainer's version is taken and `exudev generate`
   writes them again.
6. **The maintainer's own second branch** follows the same rule: it proposes, and the main branch raises and resolves.

Tests: test_issueTracker.py `testProposedResolutionsAreAppliedAfterTheMerge`, `testAContributionChangesNoNumberAndNoVersion`.
