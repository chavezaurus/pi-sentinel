var root = document.body;

var tableItems = [];

var selectedRow = null;
var paneSelect = "events";
var showComposite = false;
var showDropdown = false;

var eventsShown = "";

var eventTimeString = "";

var sentinelState = {
    startTime: { h: 21, m:  0 },
    stopTime: { h: 5, m: 30 },
    noiseThreshold: 40,
    sumThreshold: 175,
    eventsPerHour: 5,
    frameRate: 30.0,
    zenithAmplitude: 0.0,
    running: "No",
    devName: "/dev/video2",
    archivePath: "none",
    numNew: 0,
    numSaved: 0,
    numTrashed: 0,
    gpsLatitude: 0,
    gpsLongitude: 0,
    gpsTimeOffset: 0,
    startUTC: { h: 21, m: 0},
    stopUTC: { h: 5, m: 30 }
};

var playbackState = {
    year: 2019,
    month: 4,
    day: 1,
    hour: 12,
    minute: 0,
    second: 0,
    duration: 3
};

var calibrationState = {
    cameraLatitude: 35.1497,
    cameraLongitude: -106.5428,
    cameraElevation: 1690.0,
    V:      0.002278,
    S:      0.639837,
    D:     -0.000982,
    a0:  -103.264,
    E:    253.174,
    eps:    1.053,
    COPx: 986.444,
    COPy: 539.240,
    alpha:  1.570796,
    flat:   0.000000
};

var affineState = {
    c: 0.0,
    d: 0.0,
    e: 0.0,
    f: 0.0,
    g: 0.0,
    h: 0.0
};

var videoSource = null;
var videoPoster = null;
var csvFilePath = null;

var canvasContext = null;
var canvasImage = new Image();
var pixelData = null;

var landmarks = [];
var skyObjectList = [];

canvasImage.onload = function() {
    if (canvasContext != null) {
        canvasContext.drawImage(canvasImage,0,0);
        pixelData = canvasContext.getImageData(0,0,canvasImage.width,canvasImage.height);
    }
}

let CalculateAffines = function() {
    let flat = calibrationState.flat;
    let COPx = calibrationState.COPx;
    let COPy = calibrationState.COPy;
    let alpha = calibrationState.alpha;
    let dilation = Math.sqrt(1-flat);
    let K = COPx*Math.sin(alpha) + COPy*Math.cos(alpha);
    let L = COPy*Math.sin(alpha) - COPx*Math.cos(alpha);
    let c = Math.cos(alpha)*Math.cos(alpha)*dilation + Math.sin(alpha)*Math.sin(alpha)/dilation;
    let d = Math.sin(alpha)*Math.cos(alpha)*dilation - Math.sin(alpha)*Math.cos(alpha)/dilation;
    let e = -(K*Math.cos(alpha)*dilation*dilation - COPy*dilation + L*Math.sin(alpha))/dilation;
    let f = Math.sin(alpha)*Math.cos(alpha)*dilation - Math.sin(alpha)*Math.cos(alpha)/ dilation;
    let g = Math.sin(alpha)*Math.sin(alpha)*dilation + Math.cos(alpha)*Math.cos(alpha)/dilation;
    let h = -(K*Math.sin(alpha)*dilation*dilation - COPx*dilation - L*Math.cos(alpha))/dilation;

    affineState.c = c;
    affineState.d = d;
    affineState.e = e;
    affineState.f = f;
    affineState.g = g;
    affineState.h = h;
};

// For given coordinates (x, y), calculate azimuth and elevation
let PixelToAzEl = function( px, py ){
    // Apply the affine transformation to the input coordinates
    // PURPOSE: Correct elliptical image distortion

    let c = affineState.c;
    let d = affineState.d;
    let e = affineState.e;
    let f = affineState.f;
    let g = affineState.g;
    let h = affineState.h;

    let pxt = g*px + f*py + h;
    let pyt = d*px + c*py + e;

    // Now apply the Borovicka calibration equations
    // NOTE 1: The equation for "b" employs atan2(x,y), which is equivalent to (in Excel) atan2(y,x) = atan2(X,Y)
    // NOTE 2: The equation set yields azimuth measured from cardinal SOUTH

    let x = pxt - calibrationState.COPx;
    let y = pyt - calibrationState.COPy;

    let V = calibrationState.V;
    let S = calibrationState.S;
    let D = calibrationState.D;
    let a0  = calibrationState.a0  * Math.PI / 180.0;
    let E   = calibrationState.E   * Math.PI / 180.0;
    let eps = calibrationState.eps * Math.PI / 180.0;

    let r = Math.sqrt( x*x + y*y );
    let u = V*r + S*(Math.exp(D*r) - 1)
    let b = a0 - E + Math.atan2(x, y)

    var angle = b;
    var z = u;
    if (eps != 0.0) {
        z = Math.acos(Math.cos(u)*Math.cos(eps)-Math.sin(u)*Math.sin(eps)*Math.cos(b));
        sinAngle = Math.sin(b)*Math.sin(u)/Math.sin(z);
        cosAngle = (Math.cos(u)-Math.cos(eps)*Math.cos(z))/(Math.sin(eps)*Math.sin(z));
        angle = Math.atan2(sinAngle,cosAngle);
    }

    obj = {}
    obj.elev = Math.PI/2 - z;
    obj.azim = angle + E; // Measured from cardinal SOUTH.

    return obj;
}

function nelderMead2D(f, initial, step = 1, tol = 1e-6, maxIter = 200) {
    // Create simplex: three points for 2D
    let simplex = [
        [initial[0], initial[1]],
        [initial[0] + step, initial[1]],
        [initial[0], initial[1] + step]
    ];
    let values = simplex.map(p => f(p));

    for (let iter = 0; iter < maxIter; iter++) {
        // Sort simplex by function value
        let idx = [0, 1, 2].sort((a, b) => values[a] - values[b]);
        simplex = idx.map(i => simplex[i]);
        values = idx.map(i => values[i]);

        // Centroid of best two
        let centroid = [
            (simplex[0][0] + simplex[1][0]) / 2,
            (simplex[0][1] + simplex[1][1]) / 2
        ];

        // Reflection
        let reflected = [
            centroid[0] + (centroid[0] - simplex[2][0]),
            centroid[1] + (centroid[1] - simplex[2][1])
        ];
        let reflectedValue = f(reflected);

        if (reflectedValue < values[1]) {
            // Expansion
            let expanded = [
                centroid[0] + 2 * (centroid[0] - simplex[2][0]),
                centroid[1] + 2 * (centroid[1] - simplex[2][1])
            ];
            let expandedValue = f(expanded);
            if (expandedValue < reflectedValue) {
                simplex[2] = expanded;
                values[2] = expandedValue;
            } else {
                simplex[2] = reflected;
                values[2] = reflectedValue;
            }
        } else if (reflectedValue < values[2]) {
            simplex[2] = reflected;
            values[2] = reflectedValue;
        } else {
            // Contraction
            let contracted = [
                centroid[0] + 0.5 * (simplex[2][0] - centroid[0]),
                centroid[1] + 0.5 * (simplex[2][1] - centroid[1])
            ];
            let contractedValue = f(contracted);
            if (contractedValue < values[2]) {
                simplex[2] = contracted;
                values[2] = contractedValue;
            } else {
                // Shrink
                simplex[1] = [
                    simplex[0][0] + 0.5 * (simplex[1][0] - simplex[0][0]),
                    simplex[0][1] + 0.5 * (simplex[1][1] - simplex[0][1])
                ];
                simplex[2] = [
                    simplex[0][0] + 0.5 * (simplex[2][0] - simplex[0][0]),
                    simplex[0][1] + 0.5 * (simplex[2][1] - simplex[0][1])
                ];
                values[1] = f(simplex[1]);
                values[2] = f(simplex[2]);
            }
        }

        // Convergence criteria
        let diff = Math.max(
            Math.abs(values[0] - values[1]),
            Math.abs(values[0] - values[2])
        );
        if (diff < tol) break;
    }

    return {x: simplex[0], fx: values[0]};
}

let Cartesian = function( azim, elev ) {
    let r = Math.PI/2.0 - elev
    let px = r * Math.cos( azim )
    let py = r * Math.sin( azim )

    return [px, py]
}

let AzElToPixel = function(azim,elev) {
    // Convert azimuth and elevation to pixel coordinates
    // Azimuth in radians measured from cardinal SOUTH
    
    let cart0 = Cartesian( azim, elev );
    
    let f = function( v ) {
        let azel = PixelToAzEl( v[0], v[1] );
        let cart = Cartesian( azel.azim, azel.elev );

        let dx = cart[0] - cart0[0];
        let dy = cart[1] - cart0[1];

        return dx*dx + dy*dy;
    };
    
    let result = nelderMead2D(f, [900,500], step = 100);
    let vec = result.x;

    return { px: vec[0], py: vec[1] };
};

// Convert azimuth and elevation to degrees;
let RadToDeg = function( azim,  elev ) {
    // Subtract PI tp express azimuth conventionally, i.e., measured from cardinal North
    azim = azim - Math.PI;

    // Convert radians to degrees
    elev = elev * 180.0 / Math.PI;
    azim = azim * 180.0 / Math.PI;

    // Compute azimuth on the interval [0, 360)
    while (azim < 0.0) {
        azim = azim + 360.0;
    }

    while (azim >= 360.0) {
        azim = azim - 360.0;
    }

    let obj = {};
    obj.azim = azim;
    obj.elev = elev;

    return obj;
}

// Convert ezimuth and elevation to radians
let DegToRad = function( azim, elev ) {
    // Add 180 tp express azimuth measured from cardinal Soutn
    azim = azim + 180.0;

    // Convert degrees to radians
    elev = elev * Math.PI / 180.0;
    azim = azim * Math.PI / 180.0;

    let obj = {};
    obj.azim = azim;
    obj.elev = elev;

    return obj;
}

let basename = function(path) {
    return path.replace(/.*\//, '');
};

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

let Dropdown = {
    view: function() {
        let c = showDropdown ? "div.dropdown-content.show" : "div.dropdown-content";
        let running = sentinelState.running === 'Yes';

        let forceClass = running ? "a.dropdown-item" : "a.dropdown-none";
        let compoClass = running ? "a.dropdown-none" : "a.dropdown-item";

        let vidObj = videoSource !== null ? {href: videoSource, download: basename(videoSource)} : {href: "#"};
        let jpgObj = videoPoster !== null ? {href: videoPoster, download: basename(videoPoster)} : {href: "#"};
        let csvObj = csvFilePath !== null ? {href: csvFilePath, download: basename(csvFilePath)} : {href: "#"};

        return m("div.dropdown", [
            m("button.pure-button", {id: "mydrop", onclick: function(){showDropdown=!showDropdown;}}, "Actions"),
            m(c, [
                m("a.dropdown-item", {href: "#", onclick: ToggleStartStop }, "Toggle Start/Stop"),
                m(forceClass, {href: "#", onclick: ForceTrigger }, "Force Trigger"),
                m("a.dropdown-item", {href: "#", onclick: MakeComposite }, "Make Composite"),
                m("a.dropdown-item", {href: "#", onclick: Analyze }, "Analyze"),
                m(compoClass, {href: "#", onclick: MakeAverage }, "Make Star Map"),
                m("a.dropdown-item", vidObj, "Get Video"),
                m("a.dropdown-item", jpgObj, "Get Image"),
                m("a.dropdown-item", csvObj, "Get CSV File"),
                m(compoClass, {href: "#", onclick: SelfTest }, "Self Test"),
                m(compoClass, {href: "#", onclick: () => (NumberDialog.isOpen = true) }, "Recalc Times"),
                m(compoClass, {href: "#", onclick: EmptyTrash}, "Empty Trash")
            ])
        ])
    }
};

let CheckDropdown = function(e) {
    if ( !e.target.matches('#mydrop') ) {
        showDropdown = false;
    }
};

let DoHeader = {
    view: function() {
        let eventsClass    =  paneSelect === "events"    ? "button.pure-button.bselect" : "button.pure-button";
        let controlsClass  =  paneSelect === "controls"  ? "button.pure-button.bselect" : "button.pure-button";
        let playbackClass  =  paneSelect === "playback"  ? "button.pure-button.bselect" : "button.pure-button";
        let calibrateClass =  paneSelect === "calibrate" ? "button.pure-button.bselect" : "button.pure-button";
        let starsClass     =  paneSelect === "stars"     ? "button.pure-button.bselect" : "button.pure-button";

        return m("div.header-container", [
            m("h1.header-label", "Pi Sentinel"),
            m(controlsClass, {onclick: function() {
                paneSelect = "controls";
                RequestControls();
            }}, "Controls"),
            m(eventsClass, {onclick: function() {
                paneSelect = "events";
                if ( eventsShown !== "" ) {
                    UpdateEvents(eventsShown);
                }
            }}, "Events"),
            m(Dropdown),    
            m(playbackClass, {onclick: function() {
                paneSelect = "playback";
            }}, "Playback"),
            m(calibrateClass, {onclick: function() {
                paneSelect = "calibrate";
                RequestCalibration();
            }}, "Cal"),
            m(starsClass, {onclick: function() {
                paneSelect = "stars";
                RequestSkyObjects();
            }}, "Starlist"),
            m(NumberDialog)
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

let DoCalibration = function() {
    m.request({
        method: "POST",
        url: "do_calibration",
        body: { "sky_object_list": skyObjectList, "calibration_state": calibrationState }
    })
    .then( function(result) {
        if ( result.response === "OK" ) {
            calibrationState = result.calibration_state;
        } else {
            alert( result.response );
        }
    })
    .catch(function(e) {
        console.log( e.code );
    })
};

let SaveSkyObjects = function() {
    m.request({
        method: "POST",
        url: "save_sky_objects",
        body: { "sky_object_list": skyObjectList }
    })
    .then( function(result) {
        if ( result.response !== "OK" ) {
            alert( result.response );
        }
    })
    .catch(function(e) {
        console.log( e.code );
    })
};

let DoSkyTable = {
    view: function() {
        return m("div", [
            m("button.pure-button", {onclick: DoCalibration}, "Do Calibration"),
            m("button.pure-button", {onclick: UpdateStars}, "Update Stars"),
            m("button.pure-button", {onclick: function() {
                skyObjectList=[];
                SaveSkyObjects();
            }}, "Clear All")
        ])
    }
};

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

let CheckForCSV = function( path ) {
    m.request({
        method: "POST",
        url: "file_exists",
        body: { "path": path }
    })
    .then( function(result) {
        if ( result.response === "Yes") {
            csvFilePath = path;
        } else {
            csvFilePath = null;
        }
    })
    .catch(function(e) {
        console.log( e.code );
    })
};

let MakeComposite = function() {
    if ( videoSource === null ) {
        alert("No video file selected");
    }

    let path = videoSource.replace(".mp4", ".h264");

    m.request({
        method: "POST",
        url: "compose",
        body: {"path": path }
    })
    .then( function(result) {
        if ( result.response !== "OK") {
            alert( "Cannot make composite while running");
        } else {
            setTimeout( WaitForCompletion, 3000 )
        }
    })
    .catch(function(e) {
        console.log( e.code );
    })
};

let MakeAverage = function() {
    if ( videoSource === null ) {
        alert("No video file selected");
    }

    let path = videoSource.replace(".mp4", ".h264");

    m.request({
        method: "POST",
        url: "average",
        body: {"path": path }
    })
    .then( function(result) {
        if ( result.response !== "OK") {
            alert( "Cannot make average while running");
        } else {
            setTimeout( WaitForCompletion, 3000 )
        }
    })
    .catch(function(e) {
        console.log( e.code );
    })
};

let Analyze = function() {
    if ( videoSource === null ) {
        alert("No video file selected");
    }

    let path = videoSource.replace(".mp4", ".h264");
    var obj = {"path": path};

    for ( var item of landmarks ) {
        if ( item.name === "Moon" ) {
            obj["Moon"] = [ item.px, item.py ];
        }
    };

    m.request({
        method: "POST",
        url: "analyze",
        body: obj
    })
    .then( function(result) {
        if ( result.response !== "OK") {
            alert( "Cannot analyze while running");
        } else {
            csvFilePath = videoSource.replace(".mp4",".csv");
            setTimeout( WaitForCompletion, 3000 )
        }
    })
    .catch(function(e) {
        console.log( e.code );
    })
};

let EmptyTrash = function() {
    m.request({url: "empty_trash"})
    .then(function(result) {
        if ( result.response !== "OK" ) {
            alert( "Error emptying trash" );
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
        csvTest = videoSource.replace(".mp4",".csv")
        if ( e.shiftKey ) {
            posterTest = videoSource.replace(".mp4", "m.jpg");
        }
        if ( posterTest === videoPoster ) {
            showComposite = !showComposite;
        } else {
            showComposite = false;
            CheckForPoster( posterTest );
            CheckForCSV( csvTest );
        }

        landmarks = [];

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

        playbackState.year = date.getFullYear();
        playbackState.month = date.getMonth()+1;
        playbackState.day = date.getDate();
        playbackState.hour = date.getHours();
        playbackState.minute = date.getMinutes();
        playbackState.second = date.getSeconds();

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

let pixelValue = function(px,py) {
    let offset = 4*(py*canvasImage.width + px);
    return pixelData.data[offset];
}

let NearestSkyObject = function(px,py) {
    var nearest = null;
    var dsquared = 0.0;

    for ( var item of landmarks ) {
        let dx = px - item.px;
        let dy = py - item.py;
        let test = dx*dx + dy*dy;

        if ( item.circle && (nearest == null || test < dsquared) ) {
            nearest = item;
            dsquared = test;
        }
    }

    return nearest;
}

let HandleCanvasClick = function(e) {
    let eventLocation = getEventLocation(this,e);
    let px = eventLocation.x;
    let py = eventLocation.y;

    var sum = 0;
    var sumx = 0;
    var sumy = 0;

    for ( var iy = py-10; iy <= py+10; ++iy) {
        if ( iy < 0 || iy >= canvasImage.height ) {
            continue;
        }
        for ( var ix = px-10; ix <= px+10; ++ix) {
            if ( ix < 0 || ix >= canvasImage.width ) {
                continue;
            }
            let v = pixelValue(ix,iy);
            sum += v;
            sumx += ix*v;
            sumy += iy*v;
        }
    }

    var rx = 0;
    var ry = 0;

    if ( sum !== 0 ) {
        rx = sumx/sum;
        ry = sumy/sum;
    } else {
        alert("px: "+px+" py: "+py);
    }

    let nearest = NearestSkyObject(rx,ry);

    let dx = nearest.px-rx;
    let dy = nearest.py-ry;
    let dd = Math.sqrt(dx*dx+dy*dy);

    var r = confirm( nearest.name + " selected, " + dd.toFixed(1) + " pixels away.  Confirm?");

    if ( r ) {
        skyObjectList.push( {px: rx, py: ry, name: nearest.name, file: videoPoster, azim: nearest.azim, elev: nearest.elev} );
        SaveSkyObjects()
    }
};

let CalibrateImage = {
    view: function() {
        return m('div', [
            m('canvas', {
                width: 1920,
                height: 1080,
                onclick: HandleCanvasClick,
                oncreate: function(vnode) {
                    canvasContext = vnode.dom.getContext("2d");
                    canvasImage.src = videoPoster;
                    setTimeout( UpdateStars, 200);
                },
                onremove: function() {
                    canvasContext = null;
                }
            })
        ])
    }
};

var NumberDialog = {
    // State to manage the visibility of the dialog and the input value
    isOpen: false,
    value: 30,
    onSubmit: function() {
        RecalcStartStopTimes( NumberDialog.value )
        NumberDialog.isOpen = false; // Close the dialog after submission
    },
    view: function() {
        return NumberDialog.isOpen
            ? m(".modal-overlay", [
                  m(".modal", [
                      m("h2", "Minutes of Twilight"),
                      m("input[type=number]", {
                          value: NumberDialog.value,
                          oninput: function(e) {
                              NumberDialog.value = e.target.value;
                          },
                      }),
                      m("button", { onclick: NumberDialog.onSubmit }, "Submit"),
                      m("button", { onclick: () => (NumberDialog.isOpen = false) }, "Cancel"),
                  ]),
              ])
            : null; // Render nothing if the dialog is closed
    },
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

let SkyTable = {
    view: function() {
        return m("table.pure-table", [
            m("thead", m("tr", [
                m("th", "Time"),
                m("th", "Object")
            ])),
            m("tbody", skyObjectList.map( function(item,index){
                let temp = item.file.replace(eventsShown+'/s','').replace(".jpg","");
                
                return [m("tr", [
                    m("td", temp),
                    m("td", item.name)
                ])]
            }))
        ])
    }
};

let localHourMinuteToUTC = function( local ) {
    const now = new Date();
    now.setHours(local.h, local.m, 0, 0);
    return { "h": now.getUTCHours(), "m": now.getUTCMinutes() };
};

let utcToLocalHourMinute = function( utc ) {
    const now = new Date();
    now.setUTCHours(utc.h, utc.m, 0, 0);
    return { "h": now.getHours(), "m": now.getMinutes() };
};

let RecalcStartStopTimes = function( twilight ) {
    RequestCalibration();
    let body = {"twilight": Number(twilight), "lat": calibrationState.cameraLatitude, "lon": calibrationState.cameraLongitude };

    m.request({
        method: "POST",
        url: "recalc_start_stop_times",
        body: body
    })
    .then(function(result) {
        if ( result.response != "OK" ) {
            alert("RecalcStartStopTimes failed");
        } else {
            sentinelState.startTime = utcToLocalHourMinute(result.startUTC);
            sentinelState.stopTime = utcToLocalHourMinute(result.stopUTC);
        }
    })
    .catch(function(e) {
        console.log( e.code );
    })
};

let SubmitControls = function() {
    if ( sentinelState.archivePath === "" ) {
        sentinelState.archivePath = "none";
    }

    if ( sentinelState.devName === "" ) {
        sentinelState.devName = "/dev/video2";
    }

    sentinelState["startUTC"] = localHourMinuteToUTC( sentinelState.startTime );
    sentinelState["stopUTC"] = localHourMinuteToUTC( sentinelState.stopTime );

    m.request({
        method: "POST",
        url: "set_state",
        body: sentinelState
    })
    .then(function(result) {
        if ( result.response != "OK") {
            alert("Submit failed");
        } else {
            sentinelState.startTime = utcToLocalHourMinute(result.startUTC);
            sentinelState.stopTime = utcToLocalHourMinute(result.stopUTC);
        }
    })
    .catch(function(e) {
        console.log( e.code );
    })
};

let WaitForCompletion = function() {
    m.request({url: "get_running"})
    .then(function(result) {
        if ( result.response === "Yes" ) {
            setTimeout( WaitForCompletion, 3000 );
        } else {
            alert( "Action Completed" );
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
            sentinelState.startTime = utcToLocalHourMinute( result.startUTC );
            sentinelState.stopTime = utcToLocalHourMinute( result.stopUTC );
        }
    })
    .catch(function(e) {
        console.log( e.code );
    })
};

let RequestCalibration = function() {
    m.request({url: "get_calibration"})
    .then(function(result) {
        if ( Object.keys(result).length !== 0 ) {
            calibrationState = result;
        } 
    })
    .catch(function(e) {
        console.log( e.code );
    })
};

let RequestSkyObjects = function() {
    m.request({url: "get_sky_objects"})
    .then(function(result) {
        if ( Object.keys(result).length !== 0 ) {
            skyObjectList = result;
        } 
    })
    .catch(function(e) {
        console.log( e.code );
    })
};

let SelfTest = function() {
    m.request({method: "POST", url: "test"})
    .then(function(result) {
        if ( result.response != "OK") {
            alert("Self test failed");
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

let SubmitPlayback = function()
{
    var localDate = new Date();
    localDate.setFullYear(playbackState.year, playbackState.month, playbackState.day );
    localDate.setHours(playbackState.hour, playbackState.minute, playbackState.second );
    
    const body = { "year": localDate.getUTCFullYear(), "month": localDate.getUTCMonth(),
                   "day": localDate.getUTCDate(), "hour": localDate.getUTCHours(),
                   "minute": localDate.getUTCMinutes(), "second": localDate.getUTCSeconds(),
                   "duration": playbackState.duration };

    m.request({
        method: "POST",
        url: "playback",
        body: body
    })
    .then(function(result) {
        if ( result.response != "OK") {
            alert(result.response);
        }
    })
    .catch(function(e) {
        console.log( e.code );
    })
};

let PlaybackPane = {
    view: function() {
        return m("form.playback-container", {
            onsubmit: function(e) {e.preventDefault(), SubmitPlayback() }
        }, [
            m("label.pblabel", "Year"), m("label.pblabel", "Month"), m("label.pblabel", "Day"),
            m("input.pbitem[type=number]",{oninput:function(e){playbackState.year=Number(e.target.value)},value: playbackState.year}),
            m("input.pbitem[type=number]",{oninput:function(e){playbackState.month=Number(e.target.value)},value: playbackState.month}),
            m("input.pbitem[type=number]",{oninput:function(e){playbackState.day=Number(e.target.value)},value: playbackState.day}),
            m("label.pblabel", "Hour"), m("label.pblabel", "Minute"), m("label.pblabel", "Second"),
            m("input.pbitem[type=number]",{oninput:function(e){playbackState.hour=Number(e.target.value)},value: playbackState.hour}),
            m("input.pbitem[type=number]",{oninput:function(e){playbackState.minute=Number(e.target.value)},value: playbackState.minute}),
            m("input.pbitem[type=number]",{oninput:function(e){playbackState.second=Number(e.target.value)},value: playbackState.second}),
            m("label.pblabel", ""), m("label.pblabel", "Duration:"),
            m("input.pbitem[type=number]",{oninput:function(e){playbackState.duration=Number(e.target.value)},value: playbackState.duration}),
            m("button.pure-button.pbsubmit[type=submit]", "Submit")
        ])
    }
};

let ControlPane = {
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

            m("div.citem-left", "GPS Latitude:" ),
            m("div.citem-right", sentinelState.gpsLatitude ),

            m("div.citem-left", "GPS Longitude:" ),
            m("div.citem-right", sentinelState.gpsLongitude ),

            m("div.citem-left", "GPS Time Offset:" ),
            m("div.citem-right", sentinelState.gpsTimeOffset ),

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

            m("label.citem-left", "Camera Device:"),
            m("input.input[type=text]",{oninput:function(e){sentinelState.devName=e.target.value},value: sentinelState.devName}),

            m("label.citem-left", "Archive Path:"),
            m("input.input[type=text]",{oninput:function(e){sentinelState.archivePath=e.target.value},value: sentinelState.archivePath}),

            m("button.pure-button.citem-submit[type=submit]", "Submit")
        ]);
    }
};

let SaveCalibration = function() {
    m.request({
        method: "POST",
        url: "save_calibration",
        body: calibrationState
    })
    .then(function(result) {
        if ( result.response != "OK") {
            alert("Submit failed");
        }
    })
    .catch(function(e) {
        console.log( e.code );
    })
};

let CalibrationPane = {
    view: function() {
        return m("form.control-container", {
            onsubmit: function(e) {e.preventDefault(), SaveCalibration() }
        }, [
            m("div.citem-left", "Camera Latitude"),
            m("input.input[type=number,step='any']",{onchange:function(e){calibrationState.cameraLatitude=Number(e.target.value)},value: calibrationState.cameraLatitude}),
            m("div.citem-left", "Camera Longitude"),
            m("input.input[type=number,step='any']",{onchange:function(e){calibrationState.cameraLongitude=Number(e.target.value)},value: calibrationState.cameraLongitude}),
            m("div.citem-left", "Camera Elevation (m)"),
            m("input.input[type=number,step='any']",{onchange:function(e){calibrationState.cameraElevation=Number(e.target.value)},value: calibrationState.cameraElevation}),
            m("div.citem-left", "COPx"),
            m("input.input[type=number,step='any']",{onchange:function(e){calibrationState.COPx=Number(e.target.value)},value: calibrationState.COPx}),
            m("div.citem-left", "COPy"),
            m("input.input[type=number,step='any']",{onchange:function(e){calibrationState.COPy=Number(e.target.value)},value: calibrationState.COPy}),
            m("div.citem-left", "a0"),
            m("input.input[type=number,step='any']",{onchange:function(e){calibrationState.a0=Number(e.target.value)},value: calibrationState.a0}),
            m("div.citem-left", "V"),
            m("input.input[type=number,step='any']",{onchange:function(e){calibrationState.V=Number(e.target.value)},value: calibrationState.V}),
            m("div.citem-left", "S"),
            m("input.input[type=number,step='any']",{onchange:function(e){calibrationState.S=Number(e.target.value)},value: calibrationState.S}),
            m("div.citem-left", "D"),
            m("input.input[type=number,step='any']",{onchange:function(e){calibrationState.D=Number(e.target.value)},value: calibrationState.D}),
            m("div.citem-left", "E"),
            m("input.input[type=number,step='any']",{onchange:function(e){calibrationState.E=Number(e.target.value)},value: calibrationState.E}),
            m("div.citem-left", "eps"),
            m("input.input[type=number,step='any']",{onchange:function(e){calibrationState.eps=Number(e.target.value)},value: calibrationState.eps}),
            m("div.citem-left", "alpha"),
            m("input.input[type=number,step='any']",{onchange:function(e){calibrationState.alpha=Number(e.target.value)},value: calibrationState.alpha}),
            m("div.citem-left", "flat"),
            m("input.input[type=number,step='any']",{onchange:function(e){calibrationState.flat=Number(e.target.value)},value: calibrationState.flat}),

            m("button.pure-button.citem-submit[type=submit]", "Save Calibration")
        ])
    }
};

let ControlStuff = {
    view: function() {
        return m("div.form-container", [
            m("button.pure-button", {onclick: RequestControls}, "Request Update"),
            m(ControlPane)
        ]);
    }
};

let DrawLandmarks = function() {
    if ( canvasContext === null ){
        return;
    }

    canvasContext.fillStyle = "red";
    canvasContext.strokeStyle = "red";
    canvasContext.font = "20px Georgia";

    landmarks.forEach( function(item){
        if ( item.circle ) {
            canvasContext.beginPath();
            canvasContext.arc(item.px, item.py, 10, 0, 2 * Math.PI);
            canvasContext.stroke()
        
            canvasContext.fillText(item.name, item.px+12, item.py+5);    
        } else {
            canvasContext.beginPath();
            canvasContext.arc(item.px, item.py, 5, 0, 2 * Math.PI);
            canvasContext.fill()
        
            canvasContext.fillText(item.name, item.px+8, item.py+5);
        }
    })
};

let Transform = function( azElObject ) {
    let result = {};
    result.name = azElObject.name;
    result.circle = azElObject.circle;
    result.azim = azElObject.azim;
    result.elev = azElObject.elev;

    var temp = DegToRad( azElObject.azim, azElObject.elev );
    temp = AzElToPixel( temp.azim, temp.elev );

    result.px = temp.px;
    result.py = temp.py;

    return result;
}

let UpdateStars = function() {
    canvasContext.clearRect(0,0,1920,1080);
    canvasContext.drawImage(canvasImage,0,0);
    CalculateAffines();

    let directions = [
        {name: "N", azim:   0.0, elev:  0.0, circle: false },
        {name: "E", azim:  90.0, elev:  0.0, circle: false },
        {name: "S", azim: 180.0, elev:  0.0, circle: false },
        {name: "W", azim: 270.0, elev:  0.0, circle: false },
        {name: "Z", azim:   0.0, elev: 90.0, circle: false }
    ];

    let starRequest = {
        path: videoSource,
        cameraLatitude: calibrationState.cameraLatitude,
        cameraLongitude: calibrationState.cameraLongitude,
        cameraElevation: calibrationState.cameraElevation
    };

    m.request({
        method: "POST",
        url: "stars",
        body: starRequest
    })
    .then(function(result) {
        if ( result.response === "OK") {
            var temp = result.sky_objects.map( function(item){
                item.circle = true;
                return item;
            });
    
            landmarks = temp.concat(directions).map(Transform);
            DrawLandmarks();    
        } else {
            alert( result.response );
        }
    })
    .catch(function(e) {
        console.log( e.code );
    })
};
/**
 * Return the location of the element (x,y) being relative to the document.
 * 
 * @param {Element} obj Element to be located
 */
let getElementPosition = function(obj) {
    var curleft = 0, curtop = 0;
    if (obj.offsetParent) {
        do {
            curleft += obj.offsetLeft;
            curtop += obj.offsetTop;
        } while (obj = obj.offsetParent);
        return { x: curleft, y: curtop };
    }
    return undefined;
};

/** 
 * return the location of the click (or another mouse event) relative to the given element (to increase accuracy).
 * @param {DOM Object} element A dom element (button,canvas,input etc)
 * @param {DOM Event} event An event generate by an event listener.
 */
let getEventLocation = function(element,event){
    // Relies on the getElementPosition function.
    var pos = getElementPosition(element);
    
    return {
    	x: (event.pageX - pos.x),
      	y: (event.pageY - pos.y)
    };
};

let CalibrationStuff = {
    view: function() {
        return m("div.form-container", [
            m(CalibrationPane),
            m("button.pure-button", {onclick: UpdateStars}, "Update Stars")
        ]);
    }
};

let Layout = {
    view: function() {
        if ( paneSelect === "controls" ) {
            return m("div.grid-container", {onclick: CheckDropdown}, [
                m(DoHeader),
                m("div.content",  m(Video)),
                m("div.footer",   eventTimeString),
                m("div.controls", m(ControlStuff))
            ]);
        } else if ( paneSelect === "events" ) {
            return m("div.grid-container", {onclick: CheckDropdown}, [
                m(DoHeader),
                m("div.content",  m(Video)),
                m("div.footer",   eventTimeString),
                m("div.controls", m(DoTable)),
                m("div.stable",   m(Table))
            ]);
        } else if ( paneSelect === "playback" ) {
            return m("div.grid-container", {onclick: CheckDropdown}, [
                m(DoHeader),
                m("div.content",  m(Video)),
                m("div.footer",   eventTimeString),
                m("div.controls", m(PlaybackPane))
            ]);
        } else if ( paneSelect === "calibrate" ) {
            return m("div.grid-container", {onclick: CheckDropdown}, [
                m(DoHeader),
                m("div.content",  m(CalibrateImage)),
                m("div.footer",   eventTimeString),
                m("div.controls", m(CalibrationStuff))
            ]);
        } else if ( paneSelect === "stars" ) {
            return m("div.grid-container", {onclick: CheckDropdown}, [
                m(DoHeader),
                m("div.content",  m(CalibrateImage)),
                m("div.footer",   eventTimeString),
                m("div.controls", m(DoSkyTable)),
                m("div.stable",   m(SkyTable))
            ]);
        }
    }
};

m.mount(root, Layout );
