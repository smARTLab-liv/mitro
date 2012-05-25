// 10 Hz loop
function send_cmds() {
    if (keys[0] | keys[1] | keys[2] | keys[3]) {
	linear = 0;
	angular = 0;
	if (keys[0] & !keys[1]) {
	    linear = 0.7;
	}
	if (keys[1] & !keys[0]) {
	    linear = -0.15;
	}
	if (keys[2] & !keys[3]) {
	    angular = 1;
	}
	if (keys[3] & !keys[2]) {
	    angular = -1;
	}
	ws.send('cmd:' + linear + ',' + angular);
    } 
    setTimeout("send_cmds()", 100);
};          

$(document).ready(function() {
    websocket = 'ws://' + window.location.host + '/ws';
    // websocket = 'ws://%(host)s:%(port)s/ws';
    if (window.WebSocket) {
	ws = new WebSocket(websocket);
    }
    else if (window.MozWebSocket) {
	ws = MozWebSocket(websocket);
    }
    else {
	console.log('WebSocket Not Supported');
	return;
    }

    ws.onopen = function() {
	ws.send("connect");
    };

    window.onbeforeunload = function(e) {
	$('#logger').val('bye bye...\n' + $('#logger').val());
	ws.close(1000, 'disconnect');
	
	if(!e) e = window.event;
	e.stopPropagation();
	e.preventDefault();
    };

    ws.onmessage = function(evt) {
	// logging
	//$('#logger').val('<< ' + evt.data + '\n' + $('#logger').val());
	
	// parse JSON object
	obj = jQuery.parseJSON( evt.data );

	// handle runstop
	if (obj.runstop) {
	    $('#runstop').show();
	} else {
	    $('#runstop').hide();
	}

//	$('#robot_marker').attr('left', obj.location[0] + "px");
//	$('#robot_marker').attr('top', obj.location[1] + "px");

	$('#robot_marker').css({'left': ""+obj.location[0]+"px", 'top': ""+obj.location[1]+"px"});

	//$('#logger').val( );
    };

    ws.onclose = function(evt) {
	$('#logger').val($('#logger').val() + 'Connection closed by server: ' + evt.code + ' \"' + evt.reason + '\"\n');
	$('.content').toggle();
    };

    $('#skype_call').click(function() {
	ws.send('skype:' + $('#skype_user').val());
	return false;
    });

    $('#view1').click(function() {
	ws.send('view:1');
	return false;
    });

    $('#view2').click(function() {
	ws.send('view:2');
	return false;
    });

    $('#view3').click(function() {
	ws.send('view:3');
	return false;
    });

    $('#view4').click(function() {
	ws.send('view:4');
	return false;
    });

    keys = new Array(0, 0, 0, 0);

    $('#up').mousedown(function() {
	keys[0] = 1;
    }).mouseup(function() {
	keys[0] = 0;
    }).mouseleave(function() {
	keys[0] = 0;
    });

    $('#down').mousedown(function() {
	keys[1] = 1;
    }).mouseup(function() {
	keys[1] = 0;
    }).mouseleave(function() {
	keys[1] = 0;
    });

    $('#left').mousedown(function() {
	keys[2] = 1;
    }).mouseup(function() {
	keys[2] = 0;
    }).mouseleave(function() {
	keys[3] = 0;
    });

    $('#right').mousedown(function() {
	keys[3] = 1;
    }).mouseup(function() {
	keys[3] = 0;
    }).mouseleave(function() {
	keys[3] = 0;
    });

    document.onkeydown = function(e){
	keyCode = ('which' in event) ? event.which : event.keyCode;
	switch(keyCode) {
	case 38:
	    // up
	    keys[0] = 1;
	    break;
	case 40:
	    // down
	    keys[1] = 1;
	    break;
	case 37:
	    // left
	    keys[2] = 1;
	    break;
	case 39:
	    // right
	    keys[3] = 1;
	    break;
	};
    };

    document.onkeyup = function(e){
	keyCode = ('which' in event) ? event.which : event.keyCode;
	switch(keyCode) {
	case 38:
	    // up
	    keys[0] = 0;
	    break;
	case 40:
	    // down
	    keys[1] = 0;
	    break;
	case 37:
	    // left
	    keys[2] = 0;
	    break;
	case 39:
	    // right
	    keys[3] = 0;
	    break;
	};
    };

    // reset key states when focus lost
    document.onblur = function(e) {
	keys = Array(0, 0, 0, 0);
    };

    // set toggle function for logger
    $('#logger_link').click(function() {
	$('#logger').toggle();
    });

    
    // start send command loop
    setTimeout("send_cmds()", 100);

    // $('#up').button('disable').addClass('ui-state-active').removeClass('ui-state-disabled');


});


