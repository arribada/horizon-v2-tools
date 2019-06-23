cd src/build
ctest --no-compress-output --repeat-until-fail 4 --schedule-random --timeout 60 -T Test || true # Always return true to stop jenkins halting here
cd ..