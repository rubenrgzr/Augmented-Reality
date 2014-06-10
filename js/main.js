function ini(){

  google.load("visualization", "1", {packages:["corechart"]});

  var video = document.querySelector('video');
  var capture = document.getElementById('capture');
  var canvas = document.querySelector('canvas');
  var ctx = canvas.getContext('2d');
  var localMediaStream = null;
  
  

  navigator.getMedia = ( navigator.getUserMedia ||
                       navigator.webkitGetUserMedia ||
                       navigator.mozGetUserMedia ||
                       navigator.msGetUserMedia);

  function errorCallback(){
	console.log("error callback");
  }
  function snapshot() {
    if (localMediaStream) {
	 canvas.width = video.videoWidth;
     canvas.height = video.videoHeight;
	 
      ctx.drawImage(video, 0, 0);
      // "image/webp" works in Chrome.
      // Other browsers will fall back to image/png.
      document.querySelector('img').src = canvas.toDataURL('image/png');
	  
	  /*enviar la solicitud by post*/
	  $.post( "keypointsMatcher.php", { img:canvas.toDataURL('image/png') } , function( data ) {
			$("#results").html(data.hist[0]);
		});
    }
  }

  capture.addEventListener('click', snapshot, false);

  var hdConstraints = {
	video: {
	mandatory: {
	  minWidth: 1280,
	  minHeight: 720
	}
	}
	};
	
	

	//navigator.getUserMedia(hdConstraints, successCallback, errorCallback);

  
	// Not showing vendor prefixes or code that works cross-browser.
	  navigator.getMedia(
			hdConstraints, 
			function(stream) {
				video.src = window.URL.createObjectURL(stream);
				
				localMediaStream = stream;				
			}, 
			errorCallback
		);
	 } //ini

	function readFile(file, callback){
		var reader = new FileReader();
		reader.onload = callback
		reader.readAsText(file);
	}          

  function readImage(input) {
		//input = $("#image");
		
		if ( input.files && input.files[0] ) {
			console.log("asdsa")
			var FR= new FileReader();
			FR.onload = function(e) {
				 $('#snapshot').attr( "src", e.target.result );
				 //console.log(e.target.result );
				 url = "http://localhost/ar/controller/insert.php";
				  /*enviar la solicitud by post*/
				  $.post(url, { img:e.target.result } , function( data ) {
					console.log(data);
						$("#norm").html("Norma: "+data.norm);
						 drawChart( data.hist );
						 
					},"json");
			};       
			FR.readAsDataURL( input.files[0] );					
		}
	}
	
	

      function drawChart( data ) {
       
		var data2 = new google.visualization.DataTable();
		data2.addColumn('number','index');
		data2.addColumn('number','Count');
	    
		 for(i = 0; i < data.length; i++)
			data2.addRow([i+1,parseFloat(data[i])]);
		
		
		
        var options = {
          title: 'Histograma de color',
          legend: { position: 'none' },
			histogram: { bucketSize: 11 }
        };
		
		console.log(document.getElementById('chart_hist'));

        var chart = new google.visualization.BarChart(document.getElementById('chart_hist'));
        chart.draw(data2, options);
      }
		