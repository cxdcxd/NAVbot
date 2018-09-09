using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Timers;

namespace Matlab
{
    public class MatlabInterface
    {
        public bool is_connected = false;
        public CarRobot main_status = new CarRobot();
        public CarCommand main_command = new CarCommand();

        Network<CarRobot, CarCommand> main_network;
        int time_out = 50;
        Timer main_timer;
        public string version = "1.0";

        public void kill()
        {
            if (main_timer == null)
            {
                main_timer.Stop();
                main_timer.Enabled = false;
            }

            if (main_network != null)
            {
                main_network.kill();
                main_network = null;
            }

            main_timer.Stop();
            main_timer.Enabled = false;
        }

        public MatlabInterface(string ip,string port_local,string port_remote)
        {
            main_network = new Network<CarRobot, CarCommand>(ip, port_local, port_remote);
            main_network.updated += Main_network_updated;

            main_timer = new Timer();
            main_timer.Interval = 1000;
            main_timer.Elapsed += Main_timer_Elapsed;
            main_timer.Start();
            main_timer.Enabled = true;
        }

        private void Main_timer_Elapsed(object sender, ElapsedEventArgs e)
        {
            if (time_out != 0)
                time_out--;

            if ( time_out == 0 )
            {
                is_connected = false;
            }

            write();
        }

        private void Main_network_updated()
        {
            time_out = 5;
            is_connected = true;
            main_status = main_network.robot_status;
        }

        #region set

        public void setSpeed1(int value)
        {
            main_command.speed1 = value;
            write();
        }

        public void setSpeed2(int value)
        {
            main_command.speed2 = value;
            write();
        }

        public void setPosition(int value)
        {
            main_command.position = value;
            write();
        }

        public void setLed(bool value)
        {
            main_command.led = value;
            write();
        }

        public void setTarget(RVector3 target)
        {
            main_command.target_location = target;
            write();
        }

        void write()
        {
            main_network.sendMessage(main_command);
        }

        #endregion

        #region get

        public int getSpeed1()
        {
            return main_status.speed1;
        }

        public int getSpeed2()
        {
            return main_status.speed2;
        }

        public int getPosition()
        {
            return main_status.position;
        }

        public RVector3 getCurrentLocation()
        {
            return main_status.location;
        }

        public RVector3[] getPath()
        {
            return main_status.path;
        }

        #endregion

        ~MatlabInterface()
        {
            kill();
        }
    }
}
