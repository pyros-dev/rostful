var i = 0;

function timedCount() {
    i = i + 1;
    postMessage(i);
    setTimeout("timedCount()",500);
}

//TODO : all kinds of fancy stuff with roslib and rosbridge

timedCount();