var ctx = document.getElementById('error-stride').getContext('2d');
      var errorgraph = new Chart(ctx, {
        type: 'line',
        data: {
          labels: [1,2],
          datasets: [{
              data: [1,1],
              label: "Error per normalized stride",
              borderColor: "#12247D",
              fill: false
            }
          ]
        },
        options: {
          title: {
            display: true,
            text: 'Error per normalized stride'
          }
        }
      });

      var ctx2 = document.getElementById('average-stride').getContext('2d');
      var averagegraph = new Chart(ctx2, {
        type: 'line',
        data: {
          labels: [1,2],
          datasets: [{
              data: [1,1],
              label: "Baseline",
              borderColor: "#12247D",
              fill: false
            },
            {
              data: [1,1],
              label: "Average stride",
              borderColor: "#4766F6",
              fill: false
            }
          ]
        },
        options: {
          title: {
            display: true,
            text: 'Error per normalized stride'
          }
        }
      });

      eel.expose(set_error_graph);
      function set_error_graph(labels,data){
        errorgraph.data.datasets[0].data = data;
        errorgraph.data.labels = labels;
        errorgraph.update();
      }

      eel.expose(set_average_graph);
      function set_average_graph(labels,data){
        averagegraph.data.datasets[1].data = data;
        averagegraph.data.labels = labels;
        averagegraph.update();
      }

      eel.expose(set_baseline_graph);
      function set_baseline_graph(labels,data){
        averagegraph.data.datasets[0].data = data;
        averagegraph.data.labels = labels;
        averagegraph.update();
      }
      eel.expose(set_angle);
      function set_angle(x) {
        $('#angle').html(x + "&#176;");
      }

      eel.expose(register_imus);
      function register_imus(imulist, status='Ready to connect'){
        $.each(imulist, function(i, imu){
          $('#imu-list').append(`
          <div class='imu-item p-3 mb-1' data-imu='${imu[1]}'>
            <h5 class='mb-0 h6'>${imu[1]}</h5>
            <h6 class='mb-0 imu-status small'>${status}</h6>
          </div>
          `);
        });
      }

      eel.expose(update_imu_status);
      function update_imu_status(imu,status){
        let imudiv = $("div").find(`[data-imu='${imu}']`);
        imudiv.find('.imu-status').html(status);
      }

       eel.expose(feedback);
       function feedback(feedback, mode){
        $('#feedback').html(feedback);
         $('#mode').html(mode);
       }

       eel.expose(notification)
       function notification(notification){
            $('#notifications').html('');
            $('#notifications').append(`
              <div class='notification'>
                ${notification}
              </div>
            `);
       }

      $(function(){
        $('#remeasure').on('click',function(){
         eel.remeasure(true);
        });
      });

<!--      imus = [-->
<!--        ["ACdkajsldfk","WirelessIMU-5F16"],-->
<!--        ["ACdkajsldfk","WirelessIMU-6642"]-->
<!--      ];-->
<!--      register_imus(imus);-->

<!--      setTimeout(-->
<!--      function() -->
<!--      {-->
<!--        update_imu_status('WirelessIMU-5F16','Pairing');-->
<!--      }, 2000);-->
<!--      setTimeout(-->
<!--      function() -->
<!--      {-->
<!--        update_imu_status('WirelessIMU-5F16','Connected');-->
<!--      }, 5000);-->

<!--      setTimeout(-->
<!--      function() -->
<!--      {-->
<!--        update_imu_status('WirelessIMU-6642','Pairing');-->
<!--      }, 3000);-->
<!--      setTimeout(-->
<!--      function() -->
<!--      {-->
<!--        update_imu_status('WirelessIMU-6642','Connected');-->
<!--      }, 10000);-->