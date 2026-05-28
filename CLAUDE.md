# Working agreement — for Claude

Persistent project rules that **must** be honored in every session, including
fresh-context ones where conversation history was not carried over.

---

## 0. Install git hooks at session start (one Bash call)

The linewatch discipline below is enforced by a git `commit-msg` hook that
blocks commits to tracked files unless `linewatch check` passes AND the
commit message contains a `linewatch:` trailer. Hooks live in `.git/hooks/`
which is per-clone (not committed). To activate them in any fresh clone
or new session:

```
tools/install-hooks.sh
```

The script is idempotent — re-running just overwrites the destination with
the current source. Run it as the first Bash command in every session that
will edit code, before `tools/linewatch.sh begin`. If install-hooks was
skipped, you can still get a clean commit by running `linewatch check`
manually before commit and including the trailer — but the safety net is
gone, so don't skip.

## 1. linewatch discipline (mandatory before any code edit)

LLM-driven editing has a real failure mode: the Edit tool reports success
but silently truncates or grows a file far beyond what was intended. The
`tools/linewatch.sh` script catches this, but only if it is actually used.

**Rule:** before any `Edit` call that touches `MemForge2.src.c`,
`README.md`, or `quantai.ini`, the linewatch cycle is mandatory.

### Workflow for every editing session

```
1. At the START of the session (or before the first edit):
     tools/linewatch.sh begin

2. BEFORE each Edit tool call, get the EXACT line delta. Two ways:

   (a) PREFERRED — derive from real old_string / new_string content
       (no eyeball guessing, no drift):

         cat > /tmp/old.txt <<'EOF'
         <exact old_string the Edit will use>
         EOF
         cat > /tmp/new.txt <<'EOF'
         <exact new_string the Edit will use>
         EOF
         tools/linewatch.sh plan-edit /tmp/old.txt /tmp/new.txt \
                                      -f MemForge2.src.c -r "what this does"

   (b) FALLBACK — only when the change is trivial (1-2 lines) and
       counting on the fly is reliable:

         tools/linewatch.sh plan +ADDED -REMOVED -f file -r "what this does"

3. Do the Edit tool call.

4. AFTER the change is complete (or after a logical batch of related Edits):
     tools/linewatch.sh check
   This MUST exit 0. If it exits 1 (MISMATCH), STOP — do not commit, do
   not push. Investigate: was the plan wrong, or did Edit affect more
   than intended? Recover via git checkout if needed, then re-plan.

   Past failure mode (do NOT repeat): using fallback (b) for big edits,
   eyeball-guessing +12 -2 when the real result was +23 -12. Drift docs
   in commit messages were a workaround, not a fix. Always use (a) for
   anything larger than a few-line tweak.

5. Include the linewatch result in the git commit message (see below).

6. At the end of the session OR after a successful commit:
     tools/linewatch.sh reset
```

### Commit messages must reference linewatch

The user CANNOT see individual tool calls in their Claude Code UI — they
only see the final response and the git log. The only proof that linewatch
was honored is what appears in commits.

**Every commit that touches a tracked file must include a `linewatch:`
trailer line** with the actual delta. Example:

```
v0.4.30: fix BW Soak hang on dual-CCD Zen 4

<commit body>

linewatch: MemForge2.src.c +85 -12 (planned +90 -10) ✓
Co-Authored-By: ...
```

If multiple files changed, list each: `linewatch: file1 +N -M, file2 +N -M ✓`.

If the check showed any discrepancy that was investigated and accepted,
spell it out: `linewatch: ⚠ MemForge2.src.c +85 -12 (planned +90 -10);
+5 drift was an unrelated comment fix, kept in scope`.

### What does NOT require linewatch

Trivial edits where the discipline is overkill:
- Single-character / typo fixes in comments
- Editing only `.gitignore`, `CLAUDE.md`, or `DEVLOG.local.md`
- Editing the linewatch script itself

Anything that adds or removes lines of compilable C code OR public README
content gets the full cycle.

---

## 2. Version bumping — three places at once

When releasing a new version, update version strings in ALL THREE places
in a single change:

1. `MemForge2.src.c` — `replace_all` for the version literal (occurs in
   ~14 places: file header, header bars, summary tag, init log line, etc.)
2. `quantai.ini` — `Version=0.4.XX`
3. `README.md` — TWO places: `Version=0.4.XX` in the configuration example,
   AND the "Latest release: [vX.Y.Z]" link at the top of the file.

Forgetting one place is a common silent footgun. The "Bumping reminder" in
the `MemForge2.src.c` header comment exists for this reason — read it.

---

## 3. User-facing text — plain Russian, no dev jargon

In replies to the project owner and in any draft text intended for end
users (Habr commenters, GitHub issue replies), avoid:

- `g_h`, `g_w`, `g_focused_cards`, etc. — internal variable names
- `SMBIOS Type 20`, `iMC`, `MCi_STATUS`, `MSR`, `XSAVE`, `OSXSAVE` —
  acronyms users don't know
- `interleave`, `cache-line`, `MMIO`, `PPSL` — chip-architecture terms
- "Стенд", "AM5-стенд", "повтор", "воспроизвести" — dev jargon
- "Workaround в quantai.ini" — telling end users to edit config files
  defeats the point of the tool ("just works on a USB")

Rewrite into plain Russian. If a fact requires technical context, expand
inline ("чип на планке у которого данные слетают" instead of
"DRAM cell stuck-bit"). The audience is a shop technician or end user,
not a UEFI developer.

---

## 4. Don't make commitments for the user

In replies drafted FOR the user to post publicly, do NOT promise things
that cost the user time:

- ❌ "Сделаю пробную сборку именно под ваш процессор и пришлю на проверку"
- ❌ "Выкачу v0.4.30 сегодня-завтра"
- ❌ "Поправлю на следующих выходных"

These are the user's hours, not Claude's. Replace with open-ended
"посмотрю что в логе видно", "будет в ближайшее время", "если получится
прогнать с фиксом — приложите лог".

---

## 5. Never reference user-only information in public drafts

When drafting a reply to a Habr commenter, do NOT cite facts that only
appear in their attached log (motherboard model, BIOS version, etc.)
unless they wrote them in their public comment. The commenter knows
what's in their own log, but other readers see the reply as
"the author somehow knows more than I wrote". Reads as creepy.

If a fact from the log is essential, frame it as "судя по логу" /
"на вашем железе" without the specific model.

---

## 6. Visible workflow vs invisible workflow

The user does NOT see Claude's tool calls in their UI. They see only:
- The final response text
- Anything that lands in the git log
- Anything that lands on the USB stick
- Anything the program prints when run

Anything Claude does "in the background" — Bash invocations, file reads,
intermediate Edits — is invisible. Therefore every important action must
leave a trace in one of those four places, or it didn't really happen
from the user's perspective.

---

## 7. Other ongoing rules

- Do not push article drafts (`docs/habr-article*.md`) to the public repo
  without explicit approval. Use `*.local.*` suffix for anything that
  should not go to git.
- Never amend commits unless explicitly asked.
- Never `git push --force`.
- Never `git rebase`, `git reset --hard`, or any destructive operation
  without explicit permission.
- `gh` CLI is not installed; use the GitHub REST API with the token
  cached in Windows Credential Manager (`git:https://github.com`).

## 8. Deployment vocabulary — "git" always means push + release

When the project owner says **"гит" / "на гит" / "загрузи на гит"** (or the
English equivalent), it means the FULL package, every time:

1. `git push` the commit(s) to `origin/main`, AND
2. create the matching GitHub **release** for the new version (tag `vX.Y.Z`
   from the pushed commit + upload `MemForge2.efi` as the asset).

Never push a version without also cutting the release — they are one unit.
The owner stated this explicitly: *"если я говорю гит значит и то и другое,
всегда комплект."*

- **The USB flash is SEPARATE.** Only write to the stick(s) when the owner
  says "флешка" / "на флешку". "гит" does NOT imply flashing.
- A pure docs/rules commit (this file, `.gitignore`, `DEVLOG.local.md`) has no
  version to release — push it, but no GitHub release is expected for it.
