// pushToTwist() (push_link.js) against the cases in the C++ host test
// (components/PushAssist/test/push_assist_host_test.cpp), so the panel's preview and the MIB
// agree. Run: node tools/mib_panel/test/push_to_twist.test.js
'use strict';
const fs = require('fs'), path = require('path'), vm = require('vm');
const ctx = { MibLink: class {}, FrameLink: class {}, console };
vm.createContext(ctx);
vm.runInContext(fs.readFileSync(path.join(__dirname, '..', 'push_link.js'), 'utf8') + '\nthis.pushToTwist = pushToTwist; this.G = PUSH_DEFAULT_GAINS;', ctx);
const { pushToTwist, G } = ctx;
let failures = 0;
const near = (a, b) => Math.abs(a - b) <= 1e-5;
function expect(what, l, r, lin, ang, g = G) {
  const t = pushToTwist(l, r, g);
  if (!near(t.linear, lin) || !near(t.angular, ang)) { console.log(`FAIL ${what}: got ${t.linear}, ${t.angular}; want ${lin}, ${ang}`); failures++; }
}
expect('rest', 0, 0, 0, 0);
expect('inside the dead band', 9.9, -10, 0, 0);
expect('even push', 25, 25, 0.5, 0);
expect('left harder', 40, 10, 0.5, -1);
expect('right alone', 10, 40, 0.5, 1);
expect('hard push', 100, 100, 1, 0);
expect('hard push turning', 100, 40, 1, -1);
expect('pull', -40, -40, -0.3, 0);
expect('left NaN', NaN, 20, 0, 0);
expect('right NaN', 20, NaN, 0, 0);
expect('no dead band', 15, 15, 0.5, 0, { ...G, dead_band_n: 0 });
if (failures) { console.log(`${failures} failure(s)`); process.exit(1); }
console.log('pushToTwist: all passed');
