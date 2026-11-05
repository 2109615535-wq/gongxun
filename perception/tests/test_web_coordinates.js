const test = require("node:test");
const assert = require("node:assert/strict");
const fs = require("node:fs");
const path = require("node:path");
const vm = require("node:vm");

const source = fs.readFileSync(path.join(__dirname, "../rescue_vision/web/app.js"), "utf8");

function loadPage() {
  const nodes = new Map();
  function node() {
    return {
      textContent: "", children: [], attributes: {},
      addEventListener() {}, getContext() { return {}; },
      querySelector() { return this.strong ||= node(); },
      getAttribute(name) { return this.attributes[name]; },
      removeAttribute(name) { delete this.attributes[name]; },
      replaceChildren() { this.children = []; },
      append(child) { this.children.push(child); },
      get childElementCount() { return this.children.length; },
    };
  }
  function getNode(id) {
    if (!nodes.has(id)) nodes.set(id, node());
    return nodes.get(id);
  }
  const context = vm.createContext({
    document: { getElementById: getNode, createElement: node },
    AbortSignal: { timeout() {} },
    fetch: () => new Promise(() => {}),
    setInterval() {}, setTimeout() {},
  });
  vm.runInContext(source, context);
  return { render: (camera) => context.renderCamera("upper", camera), getNode };
}

function camera(detections, changes = {}) {
  return { status: "OK", age_ms: 80, selected: null, detections, ...changes };
}

function green(point, changes = {}) {
  return { name: "normal_material", confidence: .9, target_id: 13,
    position_valid: false, display_position_mm: point, ...changes };
}

test("unselected green estimates populate XY and detection labels", () => {
  const page = loadPage();
  page.render(camera([green({ x_right: -60, y_forward: 445 })]));
  assert.equal(page.getNode("upper-x").textContent, "≈ -60.0 mm");
  assert.equal(page.getNode("upper-y").textContent, "≈ 445.0 mm");
  assert.match(page.getNode("upper-quality").textContent, /旧标定地面估算.*#13/);
  assert.match(page.getNode("upper-detections").children[0].textContent, /X ≈ -60.0 mm.*Y ≈ 445.0 mm/);
});

test("validated selected coordinates retain their normal display", () => {
  const page = loadPage();
  const target = green(null, { position_mm: { x_right: 0, y_forward: 200 }, position_valid: true });
  page.render(camera([target], { selected: target }));
  assert.equal(page.getNode("upper-x").textContent, "0.0 mm");
  assert.equal(page.getNode("upper-y").textContent, "200.0 mm");
  page.render(camera([target], { selected: target, age_ms: 150 }));
  assert.equal(page.getNode("upper-x").textContent, "≈ 0.0 mm");
});

test("an unusable selected target falls back to a green estimate", () => {
  const page = loadPage();
  page.render(camera([green({ x_right: 15, y_forward: 300 })], { selected: { name: "casualty" } }));
  assert.equal(page.getNode("upper-x").textContent, "≈ 15.0 mm");
});

test("faults, stale observations and disconnects clear displayed coordinates", () => {
  const page = loadPage();
  const detections = [green({ x_right: 15, y_forward: 300 })];
  for (const changes of [{ status: "STALE" }, { status: "CAMERA_FAULT" }, { age_ms: 300 }, { age_ms: -1 }]) {
    page.render(camera(detections));
    page.render(camera(detections, changes));
    assert.equal(page.getNode("upper-x").textContent, "—");
    assert.equal(page.getNode("upper-y").textContent, "—");
  }
});

test("missing and nonfinite estimates remain unavailable", () => {
  const page = loadPage();
  for (const point of [null, { x_right: NaN, y_forward: 200 }, { x_right: 10, y_forward: Infinity }]) {
    page.render(camera([green(point)]));
    assert.equal(page.getNode("upper-x").textContent, "—");
    assert.equal(page.getNode("upper-y").textContent, "—");
  }
});

test("all green detections carry their own coordinate labels", () => {
  const page = loadPage();
  page.render(camera([green({ x_right: 0, y_forward: 0 }),
    green({ x_right: -100, y_forward: 200 }, { target_id: 15 })]));
  assert.match(page.getNode("upper-detections").children[0].textContent, /X ≈ 0.0 mm.*Y ≈ 0.0 mm/);
  assert.match(page.getNode("upper-detections").children[1].textContent, /#15.*X ≈ -100.0 mm.*Y ≈ 200.0 mm/);
});
