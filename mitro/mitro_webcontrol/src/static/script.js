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
	    $('#log').val('bye bye...\n' + $('#log').val());
	    ws.close(1000, 'disconnect');
	
	    if(!e) e = window.event;
	    e.stopPropagation();
	    e.preventDefault();
    };

    ws.onmessage = function(evt) {
        // TODO: error handling
    
	    // logging
	    //$('#log').val('<< ' + evt.data + '\n' + $('#log').val());
	    if (evt.data.indexOf("log:") == 0) {
	        $('#log').val('<< ' + evt.data.substring(4) + '\n' + $('#log').val());
	    }
	    else {
	        // parse JSON object
	        obj = jQuery.parseJSON( evt.data );

	        // handle runstop
	        if (obj.runstop) {
	            $('#runstop').removeClass().addClass("red");
	        } else {
	            $('#runstop').removeClass().addClass("green");
	        }
	        
	        // handle relais
	        if (obj.relais) {
	            $('#relais_button').removeClass("down");
	        } else {
	            $('#relais_button').addClass("down");
	        }
	        
	        //handle wifi
	        var level = -obj.wifi;
            if (level < 30) { level = 30; }
            if (level > 95) { level = 95; }
            var perc = 100 - (level-30) * 100.0/65.0;
            if (perc >= 75) { $('#wifi').removeClass().addClass("green"); }
            else if (perc >= 50) { $('#wifi').removeClass().addClass("yellow"); }
            else { $('#wifi').removeClass().addClass("red"); }
                
            // handle battery
            if (obj.battery_base >= 60) { $('#battery_robot').removeClass().addClass("green"); }
            else if (obj.battery_base >= 30) { $('#battery_robot').removeClass().addClass("yellow"); }
            else { $('#battery_robot').removeClass().addClass("red"); }

            if (obj.battery_laptop >= 60) { $('#battery_laptop').removeClass().addClass("green"); }
            else if (obj.battery_laptop >= 30) { $('#battery_laptop').removeClass().addClass("yellow"); }
            else { $('#battery_laptop').removeClass().addClass("red"); }

            // handle goal status
            if (obj.hasgoal) {
                $('#nav').removeAttr('disabled').addClass('down');
                $('#goal_marker').show()
            } else {
                $('#nav').attr('disabled', 'disabled').removeClass('down');
                $('#goal_marker').hide()
            }

            // handle assisted drive status
            if (obj.assisted) {
                $('#assisted').addClass("down");
            }
            else {
                $('#assisted').removeClass("down");
            }

            // set robot location
            $('#robot_marker').css({'left': ""+(obj.location[0]-13)+"px", 'top': ""+(obj.location[1]-13)+"px"});
            // set robot rotation
            $('#robot_marker').css("-webkit-transform", "rotate("+obj.orientation+"rad)");

            if (obj.hasOwnProperty('goal')) {
                // set goal location
                    $('#goal_marker').css({'left': ""+(obj.goal[0]-13)+"px", 'top': ""+(obj.goal[1]-26)+"px"});
            } else {
                $('#goal_marker').hide();
            }
	    }
    };
   

    ws.onclose = function(evt) {
	$('#log').val($('#log').val() + 'Connection closed by server: ' + evt.code + ' \"' + evt.reason + '\"\n');
	$('.content').toggle();
    };

    $('#skype_call').click(function() {
	ws.send('skype:' + $('#skype_user').val());
	return false;
    });

    $('#view_1').click(function() {
	ws.send('view:1');
	return false;
    });

    $('#view_2').click(function() {
	ws.send('view:2');
	return false;
    });

    $('#view_3').click(function() {
	ws.send('view:3');
	return false;
    });

    $('#view_4').click(function() {
	ws.send('view:4');
	return false;
    });
    
    function toggle_view() {
        $('.view_button').removeClass("down");
        $(this).toggleClass("down");
    };
    $('#view_1').click(toggle_view);
    $('#view_2').click(toggle_view);
    $('#view_3').click(toggle_view);
    $('#view_4').click(toggle_view);

    $('.view_button.toggle_button').click(function() {
        $(this).toggleClass("down");
    });
    
    $('#relais_button').click(function() {
        if($(this).hasClass("down")) {
            ws.send('relais:1');
        }
        else {
            ws.send('relais:0');
        }
        return false;
    });
    
    $('#assisted').click(function() {
        if($(this).hasClass("down")) {
            ws.send('assisted:0');
        }
        else {
            ws.send('assisted:1');
        }
        return false;
    });
    
    $('#costmap').click(function() {
        ws.send('recovery');
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
    
    $('#map_image').mousedown(function(e){
	if (e.which && e.which==1) {
	    var x = e.pageX - this.parentNode.offsetLeft;
	    var y = e.pageY - this.parentNode.offsetTop;
	    ws.send('goal:' + x + ':' + y);
	};
    });
    
    $('#nav').click(function() {
	ws.send('cancel_goal');
    });

    document.onkeydown = function(e){
	keyCode = ('which' in event) ? event.which : event.keyCode;
	switch(keyCode) {
	case 38:
	    // up
	    e.preventDefault();
	    keys[0] = 1;
	    $('#up').addClass("down");
	    break;
	case 40:
	    // down
	    e.preventDefault();
	    keys[1] = 1;
	    $('#down').addClass("down");
	    break;
	case 37:
	    // left
	    e.preventDefault();
	    keys[2] = 1;
	    $('#left').addClass("down");
	    break;
	case 39:
	    // right
	    e.preventDefault();
	    keys[3] = 1;
	    $('#right').addClass("down");
	    break;
	};
    };

    document.onkeyup = function(e){
	keyCode = ('which' in event) ? event.which : event.keyCode;
	switch(keyCode) {
	case 38:
	    // up
	    keys[0] = 0;
	    $('#up').removeClass("down");
	    break;
	case 40:
	    // down
	    keys[1] = 0;
	    $('#down').removeClass("down");
	    break;
	case 37:
	    // left
	    keys[2] = 0;
	    $('#left').removeClass("down");
	    break;
	case 39:
	    // right
	    keys[3] = 0;
	    $('#right').removeClass("down");
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

