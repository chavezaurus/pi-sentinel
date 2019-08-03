var root = document.body;

var tableItems = [];

var selectedRow = null;
var showControls = false;
var showComposite = false;

var eventsShown = "";

var eventTimeString = "";

let UpdateEvents = function(fromDir) {
    m.request({
        method: "POST",
        url: fromDir,
        body: tableItems
    })
    .then(function(result) {
        eventsShown = fromDir;
        tableItems = result.map( function(item) {
            return {event: item, from: fromDir, to: fromDir, selected: false};
        });

        selectedRow = null;
    })
    .catch(function(e) {
        console.log( e.code );
    })
};

let DoHeader = {
    view: function() {
        let eventsClass   =  showControls ? "button.pure-button" : "button.pure-button.bselect";
        let controlsClass = (!showControls) ? "button.pure-button" : "button.pure-button.bselect";

        return m("div.header", [
            m("h1", "Pi Sentinel"),
            m("pure-button-group", {role: "group", "aria-label": "Controls or Events Page"}, [
                m(controlsClass, {onclick: function() {
                    showControls = true;
                    RequestControls();
                }}, "Controls"),
                m(eventsClass, {onclick: function() {
                    showControls = false;
                    if ( eventsShown !== "" ) {
                        UpdateEvents(eventsShown);
                    }
                }}, "Events")
            ])
        ])
    }
};

let DoTable = {
    view: function() {
        let newClass   = eventsShown !== "new"   ? "button.pure-button" : "button.pure-button.bselect";
        let savedClass = eventsShown !== "saved" ? "button.pure-button" : "button.pure-button.bselect";
        let trashClass = eventsShown !== "trash" ? "button.pure-button" : "button.pure-button.bselect";

        return m("pure-button-group", {role: "group", "aria-label": "Event List Selection"}, [
            m(newClass,   {onclick: function() {UpdateEvents("new")}},   "New"),
            m(savedClass, {onclick: function() {UpdateEvents("saved")}}, "Saved"),
            m(trashClass, {onclick: function() {UpdateEvents("trash")}}, "Trash"),
        ])
    }
};

var videoSource = null;
var videoPoster = null;

let CheckForPoster = function( path ) {
    m.request({
        method: "POST",
        url: "file_exists",
        body: { "path": path }
    })
    .then( function(result) {
        if ( result.response === "Yes") {
            videoPoster = path;
        } else {
            videoPoster = null;
        }
    })
    .catch(function(e) {
        console.log( e.code );
    })
};

let MakeComposite = function( path ) {
    m.request({
        method: "POST",
        url: "compose",
        body: {"path": path }
    })
    .then( function(result) {
        if ( result.response !== "OK") {
            alert( "Cannot make composite while running");
        }
    })
    .catch(function(e) {
        console.log( e.code );
    })
};

let ClickTable = function(e) {
    let row = e.target.parentElement.rowIndex-1;
    let column = e.target.cellIndex;

    if ( column === 1 ) {
        var toDir = tableItems[row].to;

        if ( toDir === "new" ) {
            toDir = "trash";
        } else if ( toDir === "trash" ) {
            toDir = "saved";
        } else if ( toDir === "saved" ) {
            toDir = "new";
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
        posterTest = videoSource.replace(".mp4",".jpg");
        if ( e.shiftKey ) {
            posterTest = videoSource.replace(".mp4", "m.jpg");
        }
        if ( posterTest === videoPoster ) {
            showComposite = !showComposite;
        } else if ( e.altKey ) {
            MakeComposite( videoSource.replace(".mp4",".h264"));
        } else {
            showComposite = false;
            CheckForPoster( posterTest );
        }

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
};

let Video = {
    view: function() {
        return m('div', [
                m('img.responsive', {
                    class: showComposite ? 'yesdisplay' : 'nodisplay',
                    src: videoPoster,
                }),
                m('video.responsive', {
                    class: showComposite ? 'nodisplay' : 'yesdisplay',
                    autoplay: !showComposite,
                    // style: 'width: 1920, margin: 0.5rem',
                    controls: true,
                    src: videoSource
                })
            ])
        }
};

let Table = {
    oncreate: function() { 
        if ( eventsShown === "" ){
            UpdateEvents("new")
        }
    },
    view: function() {
        let toTable = {
            "new":   "td.whiteCell",
            "trash": "td.redCell",
            "saved": "td.goldCell" };

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
};

var sentinelState = {
    startTime: { h: 21, m:  0 },
    stopTime: { h: 5, m: 30 },
    noiseThreshold: 40,
    sumThreshold: 175,
    eventsPerHour: 5,
    frameRate: 30.0,
    zenithAmplitude: 0.0,
    running: "No",
    numNew: 0,
    numSaved: 0,
    numTrashed: 0
};

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
};

let RequestControls = function() {
    m.request({url: "get_state"})
    .then(function(result) {
        if ( Object.keys(result).length === 0 ) {
            alert( "Get State failed");
        } else {
            sentinelState = result;
        }
    })
    .catch(function(e) {
        console.log( e.code );
    })
};

let ToggleStartStop = function() {
    m.request({
        method: "POST",
        url: "toggle"
    })
    .then(function(result) {
        if ( result.response != "OK") {
            alert("Toggle failed");
        } else {
            setTimeout( RequestControls, 2000);
        }
    })
    .catch(function(e) {
        console.log( e.code );
    })
};

let ForceTrigger = function() {
    m.request({
        method: "POST",
        url: "force_trigger"
    })
    .then(function(result) {
        if ( result.response != "OK") {
            alert("Force Trigger failed");
        } else {
            setTimeout( RequestControls, 7000 );
        }
    })
    .catch(function(e) {
        console.log( e.code );
    })
};

let TimerPickers = {
    view: function() {
        return m("form.control-container",{
                onsubmit: function(e) { e.preventDefault(), SubmitControls() }
            }, [
            m("div.citem-left", "Running:"),
            m("div.citem-right", sentinelState.running ),

            m("div.citem-left", "New, Saved, Trash:"),
            m("div.citem-right", "" + sentinelState.numNew + ", " + sentinelState.numSaved + ", " + sentinelState.numTrashed),

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
};

let ControlStuff = {
    view: function() {
        return m("div.form-container", [
            m("button.pure-button", {onclick: ToggleStartStop}, "Toggle Start/Stop"),
            m("button.pure-button", {onclick: ForceTrigger},    "Force Trigger"),
            m("button.pure-button", {onclick: RequestControls}, "Request Update"),
            m(TimerPickers)
        ]);
    }
};

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
};

m.mount(root, Layout );
