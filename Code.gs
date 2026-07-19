const sheet_id = "Removed for security";

const headers = ['lastUpdate', 'outsideTemp', 'insideTemp', 'insideHumidity', 'thermostat',
                 'elapsedMinutes', 'dailyTotalMinutes', 'outsidePressure', 'insidePressure',
                 'pressureDiff', 'cyclesToday', 'coastMinutes', 'avgCycleMinutes'];

const now = new Date();
const sheetName = `${getMonthNames(now.getMonth())} ${now.getFullYear()}`;
const ss = SpreadsheetApp.openById(sheet_id);
let sheet = ss.getSheetByName(sheetName);

// Preserves the receiver's "Offline" sentinel: numeric strings become numbers,
// non-numeric text (e.g. "Offline") passes through as text instead of NaN.
// AVERAGE/charts skip text cells automatically; pandas: na_values=['Offline'].
function numOrText(v) {
  if (v === undefined || v === null || v === "") return "";
  const n = parseFloat(v);
  return isNaN(n) ? v : n;
}

function doGet(e) {

  var lastUpdate        = e.parameter.lastUpdate || "N/A";
  var outsideTemp       = numOrText(e.parameter.outsideTemp);
  var insideTemp        = numOrText(e.parameter.insideTemp);
  var insideHumidity    = numOrText(e.parameter.insideHumidity);
  var thermostat        = numOrText(e.parameter.thermostat);
  var elapsedMinutes    = numOrText(e.parameter.elapsedMinutes);
  var dailyTotalMinutes = numOrText(e.parameter.dailyTotalMinutes);
  var outsidePressure   = numOrText(e.parameter.outsidePressure);
  var insidePressure    = numOrText(e.parameter.insidePressure);
  var pressureDiff      = numOrText(e.parameter.pressureDiff);
  var cyclesToday       = numOrText(e.parameter.cyclesToday);
  var coastMinutes      = numOrText(e.parameter.coastMinutes);
  var avgCycleMinutes   = numOrText(e.parameter.avgCycleMinutes);

  // data = var to be appended to every row of Google Sheet.
  const data = [lastUpdate, outsideTemp, insideTemp, insideHumidity, thermostat,
                elapsedMinutes, dailyTotalMinutes, outsidePressure, insidePressure,
                pressureDiff, cyclesToday, coastMinutes, avgCycleMinutes];

  console.log(...data);

  // Checks for end of the month; if true creates new sheet.
  if (isEndOfMonth(now)) {
    createNewSheet(sheetName, ss, data);
  } else {
    logData(sheet, data);
  }
  return ContentService.createTextOutput(JSON.stringify(data)).setMimeType(ContentService.MimeType.JSON);
}


// Retrieves name of month.
function getMonthNames(index) {
  const months = ["January", "February", "March", "April", "May", "June", "July",
                  "August", "September", "October", "November", "December"];
  return index !== undefined ? months[index] : months;
}

// Finds date for the end of the month
function isEndOfMonth(date) {
  const endOfMonth = new Date(date.getFullYear(), date.getMonth() + 1, 0);
  return date.getDate() === endOfMonth.getDate();
}

// Creates new sheet with correct month.
function createNewSheet(sheetName, ss, data) {
  let sheet = ss.getSheetByName(sheetName);
  if (!sheet) {
    sheet = ss.insertSheet(sheetName);
    sheet.appendRow(headers);
  }
  sheet.appendRow(data);
}

// If not end of month date, opens sheet, writes data and appends data to row.
function logData(sheet, data) {
  if (!sheet) {
    const ss = SpreadsheetApp.openById(sheet_id);
    sheet = ss.insertSheet(sheetName);
    sheet.appendRow(headers);
  }
  sheet.appendRow(data);
}

// Appends a midnight summary row with yesterday's closing totals.
// Column indices updated July 19, 2026 for the 13-column schema:
//   col 7  = dailyTotalMinutes
//   col 11 = cyclesToday
//   col 13 = avgCycleMinutes
// (Previously read cols 7/8/9, which in the 10-column schema were
//  dailyTotalMinutes / outsidePressure / insidePressure -- stale layout.)
function insertMidnightSummary() {
  const now = new Date();
  const sheetName = `${getMonthNames(now.getMonth())} ${now.getFullYear()}`;
  const ss = SpreadsheetApp.openById(sheet_id);
  const sheet = ss.getSheetByName(sheetName);
  if (!sheet) return;
  const lastRow = sheet.getLastRow();

  if (lastRow < 2) return; // nothing to summarize

  // Get last totals from today's data (13 columns)
  const lastData = sheet.getRange(lastRow, 1, 1, 13).getValues()[0];
  const total  = lastData[6];   // dailyTotalMinutes  (col 7)
  const cycles = lastData[10];  // cyclesToday        (col 11)
  const avg    = lastData[12];  // avgCycleMinutes    (col 13)

  // Build summary row -- pads to the daily-total / cycle columns
  const summaryRow = [
    Utilities.formatDate(now, Session.getScriptTimeZone(), "yyyy-MM-dd 00:00"), // lastUpdate
    "", "", "", "",   // outsideTemp .. thermostat
    "",               // elapsedMinutes (blank)
    total,            // dailyTotalMinutes
    "", "", "",       // pressures, diff
    cycles,           // cyclesToday
    "",               // coastMinutes (per-cycle value, no daily meaning)
    avg               // avgCycleMinutes
  ];

  sheet.appendRow(summaryRow);
}
