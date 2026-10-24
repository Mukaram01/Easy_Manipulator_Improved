# Lifecycle reproducer: open ur5_2f_test, verify the reopened Task, close. No ROS launch.
# REPRO_BIN, REPRO_WS, REPRO_OUT, N, DOMAIN. Keeps every core, exit code and gdb backtrace.
source /opt/ros/humble/setup.bash; source $REPRO_WS/install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1 ROS_DOMAIN_ID=${DOMAIN:-46}
export QTWEBENGINE_CHROMIUM_FLAGS="${QTWEBENGINE_CHROMIUM_FLAGS:---no-sandbox --ignore-gpu-blacklist --ignore-gpu-blocklist}"
[ -n "$REOPEN" ] && MODE="--reopen --no-plan-check" || MODE="--no-plan-check"
ulimit -c unlimited
rm -rf $REPRO_OUT; mkdir -p $REPRO_OUT
crashes=0
for i in $(seq 1 $N); do
  d=$REPRO_OUT/$i; mkdir -p $d; cd $d
  timeout 300 xvfb-run -a -s "-screen 0 1920x1080x24" $REPRO_BIN --multiobject-plan-acceptance --workspace $REPRO_WS \
    --observations $REPRO_WS/src/easy_manipulation_deployment/tests/fixtures/multiobject_plan_check_ur5_2f_test.yaml \
    --smoke-output $d $MODE > $d/gui.log 2>&1
  rc=$?; res=$(python3 -c "import json;print(json.load(open('$d/$( [ -n "$REOPEN" ] && echo reopen || echo author ).json'))['result'])" 2>/dev/null)
  core=$(ls $d | grep '^core' | head -1)
  if [ -n "$core" ]; then
    crashes=$((crashes+1))
    gdb -q -batch -ex 'set pagination off' -ex 'bt 40' -ex 'info registers rip' $REPRO_BIN $d/$core > $d/backtrace.txt 2>&1
  fi
  echo "run $i exit=$rc result=$res core=${core:-none} $(grep -m1 -oE 'corrupted double-linked list|double free|free\(\): [a-z ]+|malloc\(\): [a-z ]+|Segmentation fault' $d/gui.log)"
done
echo "CRASHES=$crashes/$N"
echo REPRO_DONE
