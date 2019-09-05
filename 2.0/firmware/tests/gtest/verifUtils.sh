#!/bin/bash

postProcessValgrind() {
  name=$1
  inUse=$(sed -ne 's/^.*in use at exit: \([0-9]\+\) .*$/\1/p' ${name}-valgrind.log)
  rm -f ${name}Leak-results.xml
  exec 3>${name}Leaks-results.xml
  if [ "$inUse" != "0" ]; then
    echo >&3 '<?xml version="1.0" encoding="UTF-8"?>'
    echo >&3 '<testsuites tests="1" failures="1" disabled="0" errors="0" time="0" name="AllTests">'
    echo >&3 ' <testsuite name="Leaks" tests="1" failures="1" disabled="0" errors="0" time="0">'
    echo >&3 '   <testcase name="'${name}'" status="run" time="0" classname="Leaks">'
    echo >&3 '    <failure>'${inUse}' blocks left in use</failure>'
    echo >&3 '   </testcase>'
    echo >&3 '  </testsuite>'
    echo >&3 '</testsuites>'
  else
    echo >&3 '<?xml version="1.0" encoding="UTF-8"?>'
    echo >&3 '<testsuites tests="1" failures="0" disabled="0" errors="0" time="0" name="AllTests">'
    echo >&3 ' <testsuite name="Leaks" tests="1" failures="0" disabled="0" errors="0" time="0">'
    echo >&3 '   <testcase name="'${name}'" status="run" time="0" classname="Leaks" />'
    echo >&3 '  </testsuite>'
    echo >&3 '</testsuites>'
  fi
}

runTestSuite() {
  name=$1
  rm -f ${name}-valgrind.log
  exec 3>${name}-valgrind.log
  valgrind_cmd="valgrind --tool=memcheck --leak-check=full --show-reachable=yes --log-fd=3"
  cmd="./${name} --gtest_shuffle --gtest_repeat=10 --gtest_color=yes --gtest_output=xml:${name}-results.xml $@ ${EXTRA_TEST_ARGS}"

  if [ -n "$VALGRIND" ]; then
    $valgrind_cmd $cmd
    postProcessValgrind $name
  else
    $cmd
  fi
}
