# check-last.ps1 — quick OK/NOT-OK audit of the most recent git commit.
#
# Usage (from repo root):
#     tools\check-last.ps1
#
# Logic:
#   - If the last commit didn't touch any tracked source file
#     (MemForge2.src.c / README.md / quantai.ini) → OK (no audit needed).
#   - If it did touch one AND the commit message has a "linewatch:" line
#     → OK (discipline followed).
#   - Otherwise → NOT OK (tracked file changed but no audit trailer).
#
# Designed for the project owner to paste-and-run after every Claude
# session to verify the discipline wasn't skipped.

$ErrorActionPreference = 'Stop'

# Pull last commit hash, subject, and body.
$hash    = (git log -1 --format="%h").Trim()
$subject = (git log -1 --format="%s").Trim()
$body    = git log -1 --format="%b"
$files   = (git show -1 --name-only --format="") -join "`n"

$tracked = $files -match "MemForge2\.src\.c|README\.md|quantai\.ini"
$hasTrailer = ($body -split "`n") | Where-Object { $_ -match "^linewatch:" }

Write-Host ""
Write-Host "Last commit: $hash  $subject" -ForegroundColor Cyan
Write-Host ""

if (-not $tracked) {
    Write-Host "OK" -ForegroundColor Green -NoNewline
    Write-Host " - last commit touches no tracked source files,"
    Write-Host "     linewatch audit not required."
}
elseif ($hasTrailer) {
    Write-Host "OK" -ForegroundColor Green -NoNewline
    Write-Host " - tracked file changed AND 'linewatch:' trailer present."
    Write-Host ""
    Write-Host "     Trailer line(s) from commit body:"
    foreach ($line in $hasTrailer) {
        Write-Host "       $line" -ForegroundColor DarkGreen
    }
}
else {
    Write-Host "NOT OK" -ForegroundColor Red -NoNewline
    Write-Host " - tracked file modified but commit message has NO"
    Write-Host "          'linewatch:' trailer. This is exactly the kind"
    Write-Host "          of skip the discipline is meant to catch."
    Write-Host ""
    Write-Host "          Files changed in this commit:" -ForegroundColor Yellow
    foreach ($f in ($files -split "`n" | Where-Object { $_ })) {
        if ($f -match "MemForge2\.src\.c|README\.md|quantai\.ini") {
            Write-Host "            * $f" -ForegroundColor Red
        } else {
            Write-Host "              $f" -ForegroundColor DarkGray
        }
    }
    exit 1
}
