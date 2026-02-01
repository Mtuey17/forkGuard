// drivers.js

class Driver {
  /**
   * @param {string} name
   * @param {number} flag1
   * @param {number} flag2
   * @param {number} flag3
   * @param {number} flag4
   * @param {number} flag5
   */
  constructor(name, flag1, flag2, flag3, flag4, flag5) {
    this.name = String(name);
    this.flag1 = Number.isFinite(flag1) ? flag1 : 0;
    this.flag2 = Number.isFinite(flag2) ? flag2 : 0;
    this.flag3 = Number.isFinite(flag3) ? flag3 : 0;
    this.flag4 = Number.isFinite(flag4) ? flag4 : 0;
    this.flag5 = Number.isFinite(flag5) ? flag5 : 0;
  }
}

// ✅ Fake driver data for now (swap later with DB/API)
let drivers = [
  new Driver("Ava Chen", 2, 0, 1, 4, 1),
  new Driver("Noah Singh", 0, 6, 2, 1, 0),
  new Driver("Olivia Park", 3, 2, 1, 0, 2),
  new Driver("Liam Jones", 7, 1, 0, 2, 1),
  new Driver("Sophia Martin", 1, 0, 4, 1, 3),
  new Driver("Ethan Brooks", 5, 2, 0, 0, 1),
  new Driver("Mia Patel", 0, 1, 3, 2, 2),
  new Driver("Jackson Reed", 4, 3, 1, 1, 0),
  new Driver("Isabella Nguyen", 2, 5, 0, 1, 1),
  new Driver("Lucas Thompson", 6, 0, 2, 3, 0),
  new Driver("Charlotte King", 1, 2, 2, 0, 4),
  new Driver("Benjamin Carter", 3, 1, 5, 2, 1),
];

// Grab elements
const driverListEl = document.getElementById("driverList");
const sortByEl = document.getElementById("sortBy");
const sortDirEl = document.getElementById("sortDir");
const applySortBtn = document.getElementById("applySort");

function escapeHtml(str) {
  return String(str)
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;")
    .replaceAll("'", "&#039;");
}

function sortDrivers(sortKey, direction) {
  const dir = direction === "asc" ? 1 : -1;

  drivers.sort((a, b) => {
    if (sortKey === "name") {
      return dir * a.name.localeCompare(b.name);
    }
    return dir * ((a[sortKey] ?? 0) - (b[sortKey] ?? 0));
  });
}

function renderDrivers() {
  driverListEl.innerHTML = "";

  drivers.forEach((d) => {
    const details = document.createElement("details");
    details.className = "driver-card";

    // Optional: only allow one open at a time
    details.addEventListener("toggle", () => {
      if (details.open) {
        document.querySelectorAll(".driver-card").forEach((el) => {
          if (el !== details) el.open = false;
        });
      }
    });

    const summary = document.createElement("summary");
    summary.className = "driver-summary";
    summary.innerHTML = `
      <div class="col name">
        <span class="chev" aria-hidden="true">▸</span>
        <span class="driver-name">${escapeHtml(d.name)}</span>
      </div>
      <div class="col">${d.flag1}</div>
      <div class="col">${d.flag2}</div>
      <div class="col">${d.flag3}</div>
      <div class="col">${d.flag4}</div>
      <div class="col">${d.flag5}</div>
    `;

    const body = document.createElement("div");
    body.className = "driver-body";
    body.innerHTML = `
      <div class="graphs">
        <div class="graph-card">
          <h3>Graph Placeholder A</h3>
          <div class="graph-placeholder">[empty]</div>
        </div>
        <div class="graph-card">
          <h3>Graph Placeholder B</h3>
          <div class="graph-placeholder">[empty]</div>
        </div>
        <div class="graph-card">
          <h3>Graph Placeholder C</h3>
          <div class="graph-placeholder">[empty]</div>
        </div>
      </div>
    `;

    details.appendChild(summary);
    details.appendChild(body);
    driverListEl.appendChild(details);
  });
}

applySortBtn.addEventListener("click", () => {
  sortDrivers(sortByEl.value, sortDirEl.value);
  renderDrivers();
});

// Initial load
sortDrivers("name", "asc");
renderDrivers();
