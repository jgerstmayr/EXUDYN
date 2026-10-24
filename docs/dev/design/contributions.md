# Contributions and the version: proposed issues and proposed resolutions

The design of revision2026c step RG11.11 (#3083, maintainer 2026-10-24). A development note, not documentation: it
records the problem, what is built and the proposal for merging that waits for a decision.

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

## 3. Proposed: merging contributions without a conflict in the version

**Rule: only the maintainer's branch numbers and resolves.** A contribution carries what it would have written to the
tracker as proposals, and the merge turns them into issues and resolutions in the maintainer's sequence.

1. **Proposed resolutions.** `exudev issue propose-resolution NUMBER "release notes" --author NAME` writes
   `issues/proposed/resolutions/<date>-<author>-<number>.json` (the number of an existing issue, or the file name of a
   proposal of the same branch, which gets its number when it is raised). Nothing else of the tracker changes.
2. **Applying them.** After the merge, the maintainer runs `exudev issue apply-proposed`: first `add-proposed`, then the
   proposed resolutions in ascending issue number, each with the date of the merge, the maintainer as resolvedAuthor and
   the contributor named in the notes; each one bumps the micro version once, in the maintainer's sequence. The
   generated files are written by the tracker as for every resolve; the commit of the merge cites the issues.
3. **A check of the contribution.** `tools/checkIssues.py --contribution BASE` lists what a branch changed against its
   base (`git diff --name-only BASE...HEAD`) and refuses changes to the numbered issue files (`open/`, `closed/`,
   `archive/`), `meta.json`, `releases.json`, the version files, `CHANGELOG.md` and `docs/generated/trackerlog.md`;
   proposals pass. The maintainer runs it before a merge; in a pull request on GitHub it would be a workflow step
   (`.github/workflows/` - asked before).
4. **Conflicts in generated files** are never merged by hand: the maintainer's version is taken and `exudev generate`
   writes them again.
5. **The maintainer's own second branch** (another machine, a long experiment) follows the same rule: it proposes, and
   the main branch raises and resolves.

**Open for the decision:** whether the resolution date is the date of the merge (proposed: yes, the version describes
what the main branch contains from then on), and whether `apply-proposed` runs automatically at the next `raise` or
`resolve` as `add-proposed` does (proposed: no - a resolution changes the version and is applied deliberately).
