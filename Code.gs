const sheet_id = "See Heating System Monitor README.md";

const headers = ['lastUpdate', 'outsideTemp', 'insideTemp', 'registerTemp', 'thermostat', 'elapsedMinutes', 'dailyTotalMinutes'];

const now = new Date();
const sheetName = `${getMonthNames(now.getMonth())} ${now.getFullYear()}`;
const ss = SpreadsheetApp.openById(sheet_id);
let sheet = ss.getSheetByName(sheetName);

function doGet(e) {  
  
  //Change var name = e.paraameter.value for values to be logged
    
  var lastUpdate = e.parameter.lastUpdate || "N/A";
  var outsideTemp = e.parameter.outsideTemp ? parseFloat(e.parameter.outsideTemp) : NaN;
  var insideTemp = e.parameter.insideTemp ? parseFloat(e.parameter.insideTemp) : NaN;
  var registerTemp = e.parameter.registerTemp ? parseFloat(e.parameter.registerTemp) : NaN;
  var thermostat = e.parameter.thermostat ? parseFloat(e.parameter.thermostat) : NaN;
  var elapsedMinutes = e.parameter.elapsedMinutes ? parseFloat(e.parameter.elapsedMinutes) : NaN;
  var dailyTotalMinutes = e.parameter.dailyTotalMinutes ? parseFloat(e.parameter.dailyTotalMinutes) : NaN;

  // data = var to be appended to every row of Goole Sheet.
  const data = [lastUpdate, outsideTemp, insideTemp, registerTemp, thermostat, elapsedMinutes, dailyTotalMinutes];

  // Logs data to the console
  console.log(lastUpdate, outsideTemp, insideTemp, registerTemp, thermostat, elapsedMinutes, dailyTotalMinutes);

  //Checks for end of the month; if true creates new sheet. 
  if (isEndOfMonth(now)) {
    createNewSheet(sheetName, ss, data);
  } else {
    logData(sheet, data);
  }
  return ContentService.createTextOutput(JSON.stringify(data)).setMimeType(ContentService.MimeType.JSON);
}


//Retreves name of month.
function getMonthNames(index) {
  const months = ["January", "February", "March", "April", "May", "June", "July", "August", "September", "October", "November", "December"];
  return index !== undefined ? months[index] : months;
}

//Finds date for the end of the month
function isEndOfMonth(date) {
  const endOfMonth = new Date(date.getFullYear(), date.getMonth() + 1, 0);
  return date.getDate() === endOfMonth.getDate();
}

//Creates new sheet with correct month.
function createNewSheet(sheetName, ss, data) {
  let sheet = ss.getSheetByName(sheetName);
  if (!sheet) {
    sheet = ss.insertSheet(sheetName);
    sheet.appendRow(headers);
  }
  sheet.appendRow(data);
}

//If not end of month date, opens sheet, writes data and apends data to row.
function logData(sheet, data) {
  if (!sheet) {
    const ss = SpreadsheetApp.openById(sheet_id);
    sheet = ss.insertSheet(sheetName);
    sheet.appendRow(headers);
  }
  sheet.appendRow(data);
}

function insertMidnightSummary() {
  const now = new Date();
  const sheetName = `${getMonthNames(now.getMonth())} ${now.getFullYear()}`;
  const ss = SpreadsheetApp.openById(sheet_id);
  const sheet = ss.getSheetByName(sheetName);
  const lastRow = sheet.getLastRow();

  if (lastRow < 2) return; // nothing to summarize

  // Get last totals from today's data
  const lastData = sheet.getRange(lastRow, 1, 1, 9).getValues()[0];
  const total = lastData[6]; // dailyTotalMinutes
  const count = lastData[7]; // eventCount
  const avg = lastData[8];   // avgRunTimeMinutes

  // Build summary row
  const summaryRow = [
    Utilities.formatDate(now, Session.getScriptTimeZone(), "yyyy-MM-dd 00:00"), // lastUpdate
    "", "", "", "", // temps
    "",             // elapsedMinutes (blank)
    total,
    count,
    avg
  ];

  sheet.appendRow(summaryRow);
}
