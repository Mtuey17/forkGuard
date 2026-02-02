// drivers.js — Loads driver rows from /api/drivers.php (no MQTT)

class Driver {
  constructor({ name, weightWarning, weightError, brakeWarning, brakeError, gas }) {
    this.name = String(name ?? "");

    // force ints (PHP already casts, but keep it safe)
    this.weightWarning = Number(weightWarning) || 0;
    this.weightError = Number(weightError) || 0;
    this.brakeWarning = Number(brakeWarning) || 0;
    this.brakeError = Number(brakeError) || 0;
    this.gas = Number(gas) || 0;
  }
}

let drivers = [];

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
  if (!driverListEl) return;

  driverListEl.innerHTML = "";

  drivers.forEach((d) => {
    const details = document.createElement("details");
    details.className = "driver-card";

    // Optional: only keep one driver expanded at a time
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
      <div class="col">${d.weightWarning}</div>
      <div class="col">${d.weightError}</div>
      <div class="col">${d.brakeWarning}</div>
      <div class="col">${d.brakeError}</div>
      <div class="col">${d.gas}</div>
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

async function loadDriversFromApi() {
  // IMPORTANT: Put your PHP file at /var/www/html/api/drivers.php
  // Then this URL works from the same host:
  const res = await fetch("/api/drivers.php", { cache: "no-store" });
  if (!res.ok) throw new Error(`API error: ${res.status}`);

  const data = await res.json();

  // Convert API rows into Driver objects
  drivers = data.map((row) => new Driver(row));

  // Default sort (uses your UI if present)
  const sortKey = sortByEl?.value || "name";
  const sortDir = sortDirEl?.value || "asc";

  sortDrivers(sortKey, sortDir);
  renderDrivers();
}

// Hook up sort button
if (applySortBtn && sortByEl && sortDirEl) {
  applySortBtn.addEventListener("click", () => {
    sortDrivers(sortByEl.value, sortDirEl.value);
    renderDrivers();
  });
}
// Refresh data every 3 seconds
setInterval(() => {
  loadDriversFromApi().catch(() => {});
}, 3000);


// Initial load
loadDriversFromApi().catch((err) => {
  console.error(err);
  if (driverListEl) {
    driverListEl.innerHTML =
      `<div style="padding:12px;color:#ffd27a;">Could not load drivers from the API.</div>`;
  }
});

