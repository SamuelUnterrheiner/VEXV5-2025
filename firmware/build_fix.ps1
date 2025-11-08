# === PROS Build Fix Script ===
# 1. Cleans stray LemLib copies
# 2. Ensures proper folder structure
# 3. Validates LemLib exists
# 4. Builds project with correct include paths

Set-Location "C:\Users\samue\OneDrive\Classes\2025-2026 V5\Coding\Skills Auton\firmware"

# Remove any stray include/lemlib folder (to avoid conflicts)
if (Test-Path ".\include\lemlib") { Remove-Item -Recurse -Force ".\include\lemlib" }

# Verify proper LemLib folder exists
if (-Not (Test-Path ".\lemlib\include\lemlib")) {
    Write-Host "Error: Proper LemLib not found at lemlib/include/lemlib"
    exit
}

# Optional: Copy external source files into src (only if they don't already exist)
$externalSrc = "..\src"
if (Test-Path $externalSrc) {
    Get-ChildItem $externalSrc -Recurse -Include *.cpp | ForEach-Object {
        $dest = Join-Path ".\src" $_.Name
        if (-Not (Test-Path $dest)) { Copy-Item $_.FullName $dest }
    }
    Get-ChildItem $externalSrc -Recurse -Include *.h, *.hpp | ForEach-Object {
        $dest = Join-Path ".\include" $_.Name
        if (-Not (Test-Path $dest)) { Copy-Item $_.FullName $dest }
    }
}

# Final clean + build with correct include paths
pros make clean
pros make CXXFLAGS="-Iinclude -Ilemlib/include"
