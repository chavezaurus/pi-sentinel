var root = document.body;

var count = 0; // added a variable

var tableItems = [];

var selectedRow = null;
var showControls = false;

var eventTimeString = "";

let Increment = function(e) {
    count++;
}

let UpdateEvents = function(fromDir) {
    m.request({
        method: "POST",
        url: fromDir,
        body: tableItems
    })
    .then(function(result) {
        result.sort();
        tableItems = result.map( function(item) {
            return {event: item, from: fromDir, to: fromDir, selected: false};
        });

        selectedRow = null;
    })
    .catch(function(e) {
        console.log( e.code );
    })
}

var DoTable = {
    view: function() {
        return m("pure-button-group", {role: "group", "aria-label": "Group"}, [
            m("button.pure-button", {onclick: function() {UpdateEvents("events")}}, "Events"),
            m("button.pure-button", {onclick: function() {UpdateEvents("saved")}},  "Saved"),
            m("button.pure-button", {onclick: function() {UpdateEvents("trash")}},  "Trash"),
        ])
    }
}

var videoSource = '';

let ClickTable = function(e) {
    let row = e.target.parentElement.rowIndex-1;
    let column = e.target.cellIndex;

    if ( column === 1 ) {
        var toDir = tableItems[row].to;

        if ( toDir === "events" ) {
            toDir = "trash";
        } else if ( toDir === "trash" ) {
            toDir = "saved";
        } else if ( toDir === "saved" ) {
            toDir = "events";
        }

        tableItems[row].to = toDir;
    }

    if ( column === 0 ) {
        if ( selectedRow !== null ) {
            tableItems[selectedRow].selected = false;
        }

        selectedRow = row;
        let rowObj = tableItems[row];
        rowObj.selected = true;
        videoSource = rowObj.from+'/'+rowObj.event;

        let dateString = rowObj.event.substring(1,16);
        let year = Number(dateString.substring(0,4));
        let month = Number(dateString.substring(4,6)) - 1;
        let day = Number(dateString.substring(6,8));
        let hour = Number(dateString.substring(9,11));
        let minute = Number(dateString.substring(11,13));
        let second = Number(dateString.substring(13,15));

        var date = new Date();
        date.setUTCFullYear(year);
        date.setUTCMonth(month);
        date.setUTCDate(day);
        date.setUTCHours(hour);
        date.setUTCMinutes(minute);
        date.setUTCSeconds(second);

        eventTimeString = date.toLocaleString();
    }
}

let Video = {
    view: function() {
        return m('video', {
            autoplay: true,
            style: 'sidth: 1920, margin: 0.5rem',
            controls: true,
            src: videoSource,
        })
    }
}

let Table = {
    view: function() {
        let toTable = {
            "events": "td.whiteCell",
            "trash":  "td.redCell",
            "saved":  "td.goldCell" };

        return m("table.pure-table", [
            m("thead", m("tr", [
                m("th", "Event Videos"),
                m("th", "Move To")
            ])),
            m("tbody", {onclick: ClickTable}, tableItems.map( function(item,index){
                return [m(item.selected ? "tr.rselect" : "tr", 
                        [m("td", item.event), 
                        m(toTable[item.to], item.to)])];
            }))
        ]
        );
    }
}

let DoHeader = {
    view: function() {
        return m("div.header", [
            m("h1", "Pi Sentinel"),
            m("button.pure-button", 
                {onclick: function(){showControls=!showControls}}, 
                showControls ? "Show Event Tables" : "Show Controls Form")
        ]);
    }
}

var sentinelState = {
    startTime: { h: 21, m:  0 },
    stopTime: { h: 5, m: 30 },
    noiseThreshold: 40,
    sumThreshold: 175,
    eventsPerHour: 5,
    frameRate: 30.0,
    zenithAmplitude: 0.0,
    running: "No"
}

let SubmitControls = function() {
    m.request({
        method: "POST",
        url: "set_state",
        body: sentinelState
    })
    .then(function(result) {
        if ( result.result != "OK") {
            alert("Submit failed");
        }
    })
    .catch(function(e) {
        console.log( e.code );
    })
}

let RequestControls = function() {
    m.request({url: "get_state"})
    .then(function(result) {
        sentinelState = result;
    })
    .catch(function(e) {
        console.log( e.code );
    })
}

let ToggleStartStop = function() {
    m.request({
        method: "POST",
        url: "toggle"
    })
    .then(function(result) {
        if ( result.result != "OK") {
            alert("Toggle failed");
        }
    })
    .catch(function(e) {
        console.log( e.code );
    })
}

let ForceTrigger = function() {
    m.request({
        method: "POST",
        url: "force_trigger"
    })
    .then(function(result) {
        if ( result.result != "OK") {
            alert("Force Trigger failed");
        }
    })
    .catch(function(e) {
        console.log( e.code );
    })
}

let TimerPickers = {
    view: function() {
        return m("form.control-container",{
                onsubmit: function(e) { e.preventDefault(), SubmitControls() }
            }, [
            m("div.citem-left", "Running:"),
            m("div.citem-right", sentinelState.running ),

            m("div.citem-left", "Frame Rate:" ),
            m("div.citem-right", sentinelState.frameRate ),

            m("div.citem-left", "Zenith Amplitude:" ),
            m("div.citem-right", sentinelState.zenithAmplitude ),

            m("div.citem-left", "Auto Start Time:"), 
            m("div.citem-right", m(TimePicker, {time: sentinelState.startTime, increment: 5})),

            m("div.citem-left", "Auto Stop Time:"),   
            m("div.citem-right", m(TimePicker, {time: sentinelState.stopTime,   increment: 5})),

            m("label.citem-left", "Noise Threshold:"),
            m("input.input[type=number]",{oninput:function(e){sentinelState.noiseThreshold=Number(e.target.value)},value: sentinelState.noiseThreshold}),

            m("label.citem-left", "Trigger Threshold:"),
            m("input.input[type=number]",{oninput:function(e){sentinelState.sumThreshold=Number(e.target.value)},value: sentinelState.sumThreshold}),

            m("label.citem-left", "Max Events Per Hour:"),
            m("input.input[type=number]",{oninput:function(e){sentinelState.eventsPerHour=Number(e.target.value)},value: sentinelState.eventsPerHour}),

            m("button.pure-button.citem-submit.[type=submit]", "Submit")
        ]);
    }
}

let ControlStuff = {
    view: function() {
        return m("div.form-container", [
            m("button.pure-button", {onclick: ToggleStartStop}, "Toggle Start/Stop"),
            m("button.pure-button", {onclick: ForceTrigger},    "Force Trigger"),
            m("button.pure-button", {onclick: RequestControls}, "Request Update"),
            m(TimerPickers)
        ]);
    }
}

let Layout = {
    view: function() {
        if ( showControls ) {
            return m("div.grid-container", [
                m(DoHeader),
                m("div.content",  m(Video)),
                m("div.footer",   eventTimeString),
                m("div.controls", m(ControlStuff))
            ]);
        } else {
            return m("div.grid-container", [
                m(DoHeader),
                m("div.content",  m(Video)),
                m("div.footer",   eventTimeString),
                m("div.controls", m(DoTable)),
                m("div.stable",   m(Table))
            ]);
        }
    }
}

m.mount(root, Layout );
