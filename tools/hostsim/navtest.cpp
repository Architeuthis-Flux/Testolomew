// The nav stick decoder on the host: staggered contacts must give one press, not stray directions.
#define private public
#include "Input.h"
#undef private
#include "Console.h"
uint32_t simMillis = 0; Stream Serial; int simPinLevel[64];
static const char* names[] = {"UP","DOWN","LEFT","RIGHT","PRESS","JUP","JDOWN","JLEFT","JRIGHT","JPRESS","A","B"};
static const char* kinds[] = {"press","release","repeat","hold"};
static int fails = 0;
static void run(const char* what, int ms) { for (int t = 0; t < ms; t += 5) { simMillis += 5; input.service(); } (void)what; }
static void drain(const char* stage, const char* expect) {
    char got[256] = ""; InputEvent e;
    while (input.next(&e)) { if (e.kind == IN_REPEAT || e.kind == IN_HOLD) continue; char b[32]; snprintf(b, sizeof b, "%s:%s ", names[e.control], kinds[e.kind]); strncat(got, b, sizeof got - strlen(got) - 1); }
    bool ok = strcmp(got, expect) == 0; if (!ok) fails++;
    printf("%-34s %s  got [%s] expected [%s]\n", stage, ok ? "ok  " : "FAIL", got, expect);
}
int main() {
    console.begin(&Serial); input.begin();
    for (int p = 0; p < 64; p++) simPinLevel[p] = 1;
    input.buttons[IN_NAV_UP].pin = 10; input.buttons[IN_NAV_DOWN].pin = 11; input.buttons[IN_NAV_LEFT].pin = 12; input.buttons[IN_NAV_RIGHT].pin = 13;
    input.buttons[IN_NAV_PRESS].pin = -1; // sections 1-6: the no-push-pin fallback (three or four contacts = a press)
    run("settle", 100); drain("idle", "");
    // 1. centre push: contacts close 4 ms apart, held 300 ms, open 4 ms apart
    simPinLevel[10] = 0; run("", 5); simPinLevel[12] = 0; run("", 5); simPinLevel[11] = 0; run("", 5); simPinLevel[13] = 0; run("", 300);
    drain("push (staggered close)", "PRESS:press ");
    simPinLevel[11] = 1; run("", 5); simPinLevel[10] = 1; run("", 5); simPinLevel[13] = 1; run("", 5); simPinLevel[12] = 1; run("", 100);
    drain("push released (staggered open)", "PRESS:release ");
    // 2. one direction
    simPinLevel[10] = 0; run("", 150); drain("up", "UP:press "); simPinLevel[10] = 1; run("", 100); drain("up released", "UP:release ");
    // 3. a diagonal: two neighbours, second 6 ms later
    simPinLevel[10] = 0; run("", 5); simPinLevel[13] = 0; run("", 150); drain("up+right (diagonal)", "UP:press RIGHT:press ");
    simPinLevel[10] = 1; simPinLevel[13] = 1; run("", 100); drain("diagonal released", "UP:release RIGHT:release ");
    // 4. a bounce on one contact shorter than the debounce: nothing
    simPinLevel[11] = 0; run("", 10); simPinLevel[11] = 1; run("", 100); drain("10 ms bounce", "");
    // 5. slow release of a push: three, then two contacts stay for 60 ms - still the press, no diagonal
    for (int p = 10; p <= 13; p++) simPinLevel[p] = 0; run("", 200); drain("push again", "PRESS:press ");
    simPinLevel[10] = 1; run("", 60); simPinLevel[11] = 1; run("", 60); drain("slow release, two left", "");
    simPinLevel[12] = 1; simPinLevel[13] = 1; run("", 100); drain("slow release done", "PRESS:release ");
    // 6. a slow push: contacts 15 ms apart - longer than the debounce - is still one press, no direction first
    simPinLevel[10] = 0; run("", 15); simPinLevel[12] = 0; run("", 15); simPinLevel[11] = 0; run("", 15); simPinLevel[13] = 0; run("", 300);
    drain("slow push (15 ms apart)", "PRESS:press ");
    for (int p = 10; p <= 13; p++) simPinLevel[p] = 1; run("", 100); drain("slow push released", "PRESS:release ");
    // 7. The push contact closes on every stick movement too. Tilt: push first, direction 15 ms later -> a direction only.
    input.buttons[IN_NAV_PRESS].pin = 14; simPinLevel[14] = 1; run("", 100); drain("push pin idle", "");
    simPinLevel[14] = 0; run("", 15); simPinLevel[11] = 0; run("", 300); drain("tilt (push closes first)", "DOWN:press ");
    simPinLevel[11] = 1; run("", 10); simPinLevel[14] = 1; run("", 150); drain("tilt released", "DOWN:release ");
    // 8. Tilt: direction first, push 10 ms later.
    simPinLevel[10] = 0; run("", 10); simPinLevel[14] = 0; run("", 300); drain("tilt (direction first)", "UP:press ");
    simPinLevel[14] = 1; simPinLevel[10] = 1; run("", 150); drain("tilt released 2", "UP:release ");
    // 9. Centre push with a wobble: push, a direction contact for 30 ms, then push alone.
    simPinLevel[14] = 0; run("", 5); simPinLevel[13] = 0; run("", 30); simPinLevel[13] = 1; run("", 300); drain("push with wobble", "PRESS:press ");
    simPinLevel[14] = 1; run("", 150); drain("push released", "PRESS:release ");
    // 9b. The same with a long wobble: a direction contact for 100 ms before the push settles alone - still only the press.
    simPinLevel[14] = 0; run("", 8); simPinLevel[11] = 0; run("", 100); simPinLevel[11] = 1; run("", 300); drain("push with a 100 ms wobble", "PRESS:press ");
    simPinLevel[14] = 1; run("", 150); drain("push released 2", "PRESS:release ");
    // 9c. A short tilt (a flick of 160 ms) with the push closing too, as on this unit: a direction, not a press.
    simPinLevel[14] = 0; simPinLevel[12] = 0; run("", 160); simPinLevel[12] = 1; simPinLevel[14] = 1; run("", 150); drain("160 ms flick", "LEFT:press LEFT:release ");
    // 10. A clean centre push, and a roll from up to up+right while tilted (the diagonal follows).
    simPinLevel[14] = 0; run("", 300); drain("clean push", "PRESS:press "); simPinLevel[14] = 1; run("", 150); drain("clean push released", "PRESS:release ");
    simPinLevel[14] = 0; simPinLevel[10] = 0; run("", 200); drain("up", "UP:press "); simPinLevel[13] = 0; run("", 200); drain("rolled to up+right", "RIGHT:press ");
    simPinLevel[10] = 1; simPinLevel[13] = 1; simPinLevel[14] = 1; run("", 150); drain("roll released", "UP:release RIGHT:release ");
    printf("%s\n", fails ? "FAILURES" : "all ok");
    return fails;
}
