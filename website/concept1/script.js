// script.js

class Driver {
  constructor(name, id, Sscore, incidents, SscoreOverTime) {
    this.name = name;
    this.id = id;
    this.incidents = incidents;
    this.safetyScore = Sscore;
    this.scoreOverTime = SscoreOverTime;
  }
}

const drivers = [
  new Driver("lebron james", "1", 88, 1, [80, 85, 90, 95, 87, 88, 92]),
  new Driver("tim stutzle", "2", 92, 0, [88, 91, 89, 94, 96, 90, 92]),
  new Driver("matt tuer", "3", 76, 2, [70, 60, 78, 80, 74, 76, 79]),
  new Driver("jake sanderson", "4", 89, 2, [70, 75, 78, 80, 74, 76, 79])
];

const driverList = document.getElementById("driverList");

drivers.forEach((driver, index) => {
  const card = document.createElement("div");
  card.className = "driver-card bg-white p-4 rounded-xl shadow cursor-pointer";


  card.innerHTML = `
    <div class="flex justify-between items-center">
      <div>
        <h2 class="text-xl font-semibold text-gray-800">${driver.name}</h2>
        <p class="text-sm text-gray-500">ID: ${driver.id}</p>
      </div>
      <span class="text-gray-600">▼</span>
    </div>

    <div id="details-${index}" class="hidden mt-4 border-t pt-4">
      <div class="grid grid-cols-3 gap-4 mb-4">
        <div class="bg-gray-50 p-3 rounded-lg text-center">
          <p class="text-sm text-gray-500">Safety Score</p>
          <p class="text-xl font-bold text-green-600">${driver.safetyScore}%</p>
        </div>
        <div class="bg-gray-50 p-3 rounded-lg text-center">
          <p class="text-sm text-gray-500">Incidents</p>
          <p class="text-xl font-bold text-blue-600">${driver.incidents}</p>
        </div>
      </div>
      <canvas id="chart-${index}" height="100"></canvas>
    </div>
  `;


  card.addEventListener("click", () => {
    const details = document.getElementById(`details-${index}`);
    const icon = card.querySelector("span");
    const isHidden = details.classList.contains("hidden");

    // collapse all others
    document.querySelectorAll("[id^='details-']").forEach((d) => d.classList.add("hidden"));
    document.querySelectorAll(".driver-card span").forEach((i) => (i.textContent = "▼"));

    // expand this one
    if (isHidden) {
      details.classList.remove("hidden");
      icon.textContent = "▲";
      renderChart(`chart-${index}`, driver.scoreOverTime);
    }
  });

  driverList.appendChild(card);
});

function renderChart(canvasId, data) {
  const ctx = document.getElementById(canvasId).getContext("2d");
  const existingChart = Chart.getChart(ctx);
  if (existingChart) existingChart.destroy();

  
  const pointColors = data.map(value => value < 65 ? "red" : "#2563EB");

  new Chart(ctx, {
    type: "line",
    data: {
      labels: ["Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun"],
      datasets: [
        {
          label: "Safety Score over time",
          data,
          borderColor: "#2563EB",    
          borderWidth: 2,
          fill: false,
          tension: 0.3,
          pointBackgroundColor: pointColors,  
          pointBorderColor: pointColors,
          pointRadius: 6,                     
          pointHoverRadius: 8
        }
      ]
    },
    options: {
      scales: {
        y: { beginAtZero: true, max: 100 }
      },
      plugins: {
        legend: { display: false }
      }
    }
  });
}


