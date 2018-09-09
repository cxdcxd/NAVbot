namespace main
{
    partial class frmMain
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            this.img_map = new System.Windows.Forms.PictureBox();
            this.label2 = new System.Windows.Forms.Label();
            this.txt_speed1 = new System.Windows.Forms.TextBox();
            this.label3 = new System.Windows.Forms.Label();
            this.txt_speed2 = new System.Windows.Forms.TextBox();
            this.label4 = new System.Windows.Forms.Label();
            this.txt_position = new System.Windows.Forms.TextBox();
            this.Load = new System.Windows.Forms.Label();
            this.txt_load = new System.Windows.Forms.TextBox();
            this.txt_set_speed_1 = new System.Windows.Forms.TextBox();
            this.txt_set_speed_2 = new System.Windows.Forms.TextBox();
            this.txt_set_position = new System.Windows.Forms.TextBox();
            this.btn_led = new System.Windows.Forms.Button();
            this.label6 = new System.Windows.Forms.Label();
            this.txt_voltage = new System.Windows.Forms.TextBox();
            this.label7 = new System.Windows.Forms.Label();
            this.txt_sensor = new System.Windows.Forms.TextBox();
            this.label8 = new System.Windows.Forms.Label();
            this.txt_alarm = new System.Windows.Forms.TextBox();
            this.btn_reset1 = new System.Windows.Forms.Button();
            this.track_speed_2 = new System.Windows.Forms.TrackBar();
            this.track_position = new System.Windows.Forms.TrackBar();
            this.lst_points = new System.Windows.Forms.ListBox();
            this.label9 = new System.Windows.Forms.Label();
            this.label10 = new System.Windows.Forms.Label();
            this.track_speed_1 = new System.Windows.Forms.TrackBar();
            this.panel_robot = new System.Windows.Forms.Panel();
            this.panel_matlab = new System.Windows.Forms.Panel();
            this.btn_set_target = new System.Windows.Forms.Button();
            this.btn_navigation = new System.Windows.Forms.Button();
            this.btn_cancel = new System.Windows.Forms.Button();
            this.txt_x = new System.Windows.Forms.TextBox();
            this.txt_y = new System.Windows.Forms.TextBox();
            this.txt_theta = new System.Windows.Forms.TextBox();
            this.label1 = new System.Windows.Forms.Label();
            this.label13 = new System.Windows.Forms.Label();
            this.label14 = new System.Windows.Forms.Label();
            this.main_timer = new System.Windows.Forms.Timer(this.components);
            this.rad_laser = new System.Windows.Forms.RadioButton();
            this.btn_reset_map = new System.Windows.Forms.Button();
            this.label12 = new System.Windows.Forms.Label();
            this.txt_robot_ip = new System.Windows.Forms.TextBox();
            this.label15 = new System.Windows.Forms.Label();
            this.txt_matlab_ip = new System.Windows.Forms.TextBox();
            this.btn_config = new System.Windows.Forms.Button();
            this.label16 = new System.Windows.Forms.Label();
            this.txt_current_position = new System.Windows.Forms.TextBox();
            this.label17 = new System.Windows.Forms.Label();
            this.txt_current_goal_position = new System.Windows.Forms.TextBox();
            this.btn_reset2 = new System.Windows.Forms.Button();
            this.btn_reset3 = new System.Windows.Forms.Button();
            this.btn_stop = new System.Windows.Forms.Button();
            this.map_timer = new System.Windows.Forms.Timer(this.components);
            this.nav_timer = new System.Windows.Forms.Timer(this.components);
            this.matlab = new System.Windows.Forms.Timer(this.components);
            ((System.ComponentModel.ISupportInitialize)(this.img_map)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.track_speed_2)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.track_position)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.track_speed_1)).BeginInit();
            this.SuspendLayout();
            // 
            // img_map
            // 
            this.img_map.Location = new System.Drawing.Point(299, 24);
            this.img_map.Margin = new System.Windows.Forms.Padding(2, 2, 2, 2);
            this.img_map.Name = "img_map";
            this.img_map.Size = new System.Drawing.Size(384, 416);
            this.img_map.SizeMode = System.Windows.Forms.PictureBoxSizeMode.StretchImage;
            this.img_map.TabIndex = 0;
            this.img_map.TabStop = false;
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(9, 7);
            this.label2.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(59, 15);
            this.label2.TabIndex = 4;
            this.label2.Text = "Speed 1 :";
            // 
            // txt_speed1
            // 
            this.txt_speed1.Font = new System.Drawing.Font("Microsoft Sans Serif", 19.8F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.txt_speed1.Location = new System.Drawing.Point(11, 24);
            this.txt_speed1.Margin = new System.Windows.Forms.Padding(2, 2, 2, 2);
            this.txt_speed1.Name = "txt_speed1";
            this.txt_speed1.ReadOnly = true;
            this.txt_speed1.Size = new System.Drawing.Size(120, 45);
            this.txt_speed1.TabIndex = 3;
            this.txt_speed1.Text = "0";
            this.txt_speed1.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(166, 7);
            this.label3.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(59, 15);
            this.label3.TabIndex = 6;
            this.label3.Text = "Speed 2 :";
            // 
            // txt_speed2
            // 
            this.txt_speed2.Font = new System.Drawing.Font("Microsoft Sans Serif", 19.8F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.txt_speed2.Location = new System.Drawing.Point(169, 24);
            this.txt_speed2.Margin = new System.Windows.Forms.Padding(2, 2, 2, 2);
            this.txt_speed2.Name = "txt_speed2";
            this.txt_speed2.ReadOnly = true;
            this.txt_speed2.Size = new System.Drawing.Size(120, 45);
            this.txt_speed2.TabIndex = 5;
            this.txt_speed2.Text = "0";
            this.txt_speed2.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(9, 63);
            this.label4.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(57, 15);
            this.label4.TabIndex = 8;
            this.label4.Text = "Position :";
            // 
            // txt_position
            // 
            this.txt_position.Font = new System.Drawing.Font("Microsoft Sans Serif", 19.8F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.txt_position.Location = new System.Drawing.Point(11, 81);
            this.txt_position.Margin = new System.Windows.Forms.Padding(2, 2, 2, 2);
            this.txt_position.Name = "txt_position";
            this.txt_position.ReadOnly = true;
            this.txt_position.Size = new System.Drawing.Size(278, 45);
            this.txt_position.TabIndex = 7;
            this.txt_position.Text = "0";
            this.txt_position.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // Load
            // 
            this.Load.AutoSize = true;
            this.Load.Location = new System.Drawing.Point(9, 116);
            this.Load.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.Load.Name = "Load";
            this.Load.Size = new System.Drawing.Size(41, 15);
            this.Load.TabIndex = 10;
            this.Load.Text = "Load :";
            // 
            // txt_load
            // 
            this.txt_load.Font = new System.Drawing.Font("Microsoft Sans Serif", 19.8F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.txt_load.Location = new System.Drawing.Point(11, 132);
            this.txt_load.Margin = new System.Windows.Forms.Padding(2, 2, 2, 2);
            this.txt_load.Name = "txt_load";
            this.txt_load.ReadOnly = true;
            this.txt_load.Size = new System.Drawing.Size(278, 45);
            this.txt_load.TabIndex = 9;
            this.txt_load.Text = "0";
            this.txt_load.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // txt_set_speed_1
            // 
            this.txt_set_speed_1.Font = new System.Drawing.Font("Microsoft Sans Serif", 19.8F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.txt_set_speed_1.Location = new System.Drawing.Point(602, 456);
            this.txt_set_speed_1.Margin = new System.Windows.Forms.Padding(2, 2, 2, 2);
            this.txt_set_speed_1.Name = "txt_set_speed_1";
            this.txt_set_speed_1.ReadOnly = true;
            this.txt_set_speed_1.Size = new System.Drawing.Size(82, 45);
            this.txt_set_speed_1.TabIndex = 18;
            this.txt_set_speed_1.Text = "128";
            this.txt_set_speed_1.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // txt_set_speed_2
            // 
            this.txt_set_speed_2.Font = new System.Drawing.Font("Microsoft Sans Serif", 19.8F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.txt_set_speed_2.Location = new System.Drawing.Point(602, 506);
            this.txt_set_speed_2.Margin = new System.Windows.Forms.Padding(2, 2, 2, 2);
            this.txt_set_speed_2.Name = "txt_set_speed_2";
            this.txt_set_speed_2.ReadOnly = true;
            this.txt_set_speed_2.Size = new System.Drawing.Size(82, 45);
            this.txt_set_speed_2.TabIndex = 19;
            this.txt_set_speed_2.Text = "128";
            this.txt_set_speed_2.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // txt_set_position
            // 
            this.txt_set_position.Font = new System.Drawing.Font("Microsoft Sans Serif", 19.8F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.txt_set_position.Location = new System.Drawing.Point(602, 557);
            this.txt_set_position.Margin = new System.Windows.Forms.Padding(2, 2, 2, 2);
            this.txt_set_position.Name = "txt_set_position";
            this.txt_set_position.ReadOnly = true;
            this.txt_set_position.Size = new System.Drawing.Size(82, 45);
            this.txt_set_position.TabIndex = 20;
            this.txt_set_position.Text = "128";
            this.txt_set_position.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // btn_led
            // 
            this.btn_led.Location = new System.Drawing.Point(902, 176);
            this.btn_led.Margin = new System.Windows.Forms.Padding(2, 2, 2, 2);
            this.btn_led.Name = "btn_led";
            this.btn_led.Size = new System.Drawing.Size(214, 33);
            this.btn_led.TabIndex = 21;
            this.btn_led.Text = "LED ( Set ON )";
            this.btn_led.UseVisualStyleBackColor = true;
            this.btn_led.Click += new System.EventHandler(this.btn_led_Click);
            // 
            // label6
            // 
            this.label6.AutoSize = true;
            this.label6.Location = new System.Drawing.Point(9, 175);
            this.label6.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(54, 15);
            this.label6.TabIndex = 23;
            this.label6.Text = "Voltage :";
            // 
            // txt_voltage
            // 
            this.txt_voltage.Font = new System.Drawing.Font("Microsoft Sans Serif", 19.8F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.txt_voltage.Location = new System.Drawing.Point(11, 191);
            this.txt_voltage.Margin = new System.Windows.Forms.Padding(2, 2, 2, 2);
            this.txt_voltage.Name = "txt_voltage";
            this.txt_voltage.ReadOnly = true;
            this.txt_voltage.Size = new System.Drawing.Size(278, 45);
            this.txt_voltage.TabIndex = 22;
            this.txt_voltage.Text = "0";
            this.txt_voltage.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // label7
            // 
            this.label7.AutoSize = true;
            this.label7.Location = new System.Drawing.Point(11, 230);
            this.label7.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(52, 15);
            this.label7.TabIndex = 25;
            this.label7.Text = "Sensor :";
            // 
            // txt_sensor
            // 
            this.txt_sensor.Font = new System.Drawing.Font("Microsoft Sans Serif", 19.8F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.txt_sensor.Location = new System.Drawing.Point(11, 246);
            this.txt_sensor.Margin = new System.Windows.Forms.Padding(2, 2, 2, 2);
            this.txt_sensor.Name = "txt_sensor";
            this.txt_sensor.ReadOnly = true;
            this.txt_sensor.Size = new System.Drawing.Size(278, 45);
            this.txt_sensor.TabIndex = 24;
            this.txt_sensor.Text = "0";
            this.txt_sensor.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // label8
            // 
            this.label8.AutoSize = true;
            this.label8.Location = new System.Drawing.Point(9, 285);
            this.label8.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label8.Name = "label8";
            this.label8.Size = new System.Drawing.Size(45, 15);
            this.label8.TabIndex = 27;
            this.label8.Text = "Alarm :";
            // 
            // txt_alarm
            // 
            this.txt_alarm.Font = new System.Drawing.Font("Microsoft Sans Serif", 19.8F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.txt_alarm.Location = new System.Drawing.Point(11, 301);
            this.txt_alarm.Margin = new System.Windows.Forms.Padding(2, 2, 2, 2);
            this.txt_alarm.Name = "txt_alarm";
            this.txt_alarm.ReadOnly = true;
            this.txt_alarm.Size = new System.Drawing.Size(278, 45);
            this.txt_alarm.TabIndex = 26;
            this.txt_alarm.Text = "0";
            this.txt_alarm.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // btn_reset1
            // 
            this.btn_reset1.Location = new System.Drawing.Point(11, 463);
            this.btn_reset1.Margin = new System.Windows.Forms.Padding(2, 2, 2, 2);
            this.btn_reset1.Name = "btn_reset1";
            this.btn_reset1.Size = new System.Drawing.Size(66, 33);
            this.btn_reset1.TabIndex = 28;
            this.btn_reset1.Text = "Reset";
            this.btn_reset1.UseVisualStyleBackColor = true;
            this.btn_reset1.Click += new System.EventHandler(this.btn_reset_Click);
            // 
            // track_speed_2
            // 
            this.track_speed_2.Location = new System.Drawing.Point(99, 509);
            this.track_speed_2.Margin = new System.Windows.Forms.Padding(2, 2, 2, 2);
            this.track_speed_2.Maximum = 250;
            this.track_speed_2.Minimum = 5;
            this.track_speed_2.Name = "track_speed_2";
            this.track_speed_2.Size = new System.Drawing.Size(500, 56);
            this.track_speed_2.TabIndex = 29;
            this.track_speed_2.TickStyle = System.Windows.Forms.TickStyle.Both;
            this.track_speed_2.Value = 128;
            this.track_speed_2.Scroll += new System.EventHandler(this.track_speed_2_Scroll);
            // 
            // track_position
            // 
            this.track_position.Location = new System.Drawing.Point(180, 554);
            this.track_position.Margin = new System.Windows.Forms.Padding(2, 2, 2, 2);
            this.track_position.Maximum = 170;
            this.track_position.Minimum = 80;
            this.track_position.Name = "track_position";
            this.track_position.Size = new System.Drawing.Size(320, 56);
            this.track_position.TabIndex = 30;
            this.track_position.TickStyle = System.Windows.Forms.TickStyle.Both;
            this.track_position.Value = 128;
            this.track_position.Scroll += new System.EventHandler(this.track_position_Scroll);
            // 
            // lst_points
            // 
            this.lst_points.FormattingEnabled = true;
            this.lst_points.Location = new System.Drawing.Point(688, 24);
            this.lst_points.Margin = new System.Windows.Forms.Padding(2, 2, 2, 2);
            this.lst_points.Name = "lst_points";
            this.lst_points.Size = new System.Drawing.Size(206, 420);
            this.lst_points.TabIndex = 31;
            // 
            // label9
            // 
            this.label9.AutoSize = true;
            this.label9.Location = new System.Drawing.Point(686, 7);
            this.label9.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label9.Name = "label9";
            this.label9.Size = new System.Drawing.Size(47, 15);
            this.label9.TabIndex = 32;
            this.label9.Text = "Points :";
            // 
            // label10
            // 
            this.label10.AutoSize = true;
            this.label10.Location = new System.Drawing.Point(297, 7);
            this.label10.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label10.Name = "label10";
            this.label10.Size = new System.Drawing.Size(44, 15);
            this.label10.TabIndex = 33;
            this.label10.Text = "Laser :";
            // 
            // track_speed_1
            // 
            this.track_speed_1.Location = new System.Drawing.Point(99, 459);
            this.track_speed_1.Margin = new System.Windows.Forms.Padding(2, 2, 2, 2);
            this.track_speed_1.Maximum = 250;
            this.track_speed_1.Minimum = 5;
            this.track_speed_1.Name = "track_speed_1";
            this.track_speed_1.Size = new System.Drawing.Size(500, 56);
            this.track_speed_1.TabIndex = 34;
            this.track_speed_1.TickStyle = System.Windows.Forms.TickStyle.Both;
            this.track_speed_1.Value = 128;
            this.track_speed_1.Scroll += new System.EventHandler(this.track_speed_1_Scroll);
            // 
            // panel_robot
            // 
            this.panel_robot.BackColor = System.Drawing.Color.DarkRed;
            this.panel_robot.Location = new System.Drawing.Point(902, 24);
            this.panel_robot.Margin = new System.Windows.Forms.Padding(2, 2, 2, 2);
            this.panel_robot.Name = "panel_robot";
            this.panel_robot.Size = new System.Drawing.Size(34, 37);
            this.panel_robot.TabIndex = 35;
            // 
            // panel_matlab
            // 
            this.panel_matlab.BackColor = System.Drawing.Color.DarkRed;
            this.panel_matlab.Location = new System.Drawing.Point(902, 79);
            this.panel_matlab.Margin = new System.Windows.Forms.Padding(2, 2, 2, 2);
            this.panel_matlab.Name = "panel_matlab";
            this.panel_matlab.Size = new System.Drawing.Size(34, 37);
            this.panel_matlab.TabIndex = 36;
            // 
            // btn_set_target
            // 
            this.btn_set_target.Location = new System.Drawing.Point(922, 456);
            this.btn_set_target.Margin = new System.Windows.Forms.Padding(2, 2, 2, 2);
            this.btn_set_target.Name = "btn_set_target";
            this.btn_set_target.Size = new System.Drawing.Size(176, 33);
            this.btn_set_target.TabIndex = 40;
            this.btn_set_target.Text = "Set Target";
            this.btn_set_target.UseVisualStyleBackColor = true;
            this.btn_set_target.Click += new System.EventHandler(this.btn_set_target_Click);
            // 
            // btn_navigation
            // 
            this.btn_navigation.Location = new System.Drawing.Point(922, 506);
            this.btn_navigation.Margin = new System.Windows.Forms.Padding(2, 2, 2, 2);
            this.btn_navigation.Name = "btn_navigation";
            this.btn_navigation.Size = new System.Drawing.Size(176, 33);
            this.btn_navigation.TabIndex = 41;
            this.btn_navigation.Text = "Navigate";
            this.btn_navigation.UseVisualStyleBackColor = true;
            this.btn_navigation.Click += new System.EventHandler(this.btn_navigation_Click);
            // 
            // btn_cancel
            // 
            this.btn_cancel.Location = new System.Drawing.Point(923, 557);
            this.btn_cancel.Margin = new System.Windows.Forms.Padding(2, 2, 2, 2);
            this.btn_cancel.Name = "btn_cancel";
            this.btn_cancel.Size = new System.Drawing.Size(176, 33);
            this.btn_cancel.TabIndex = 42;
            this.btn_cancel.Text = "Cancel";
            this.btn_cancel.UseVisualStyleBackColor = true;
            this.btn_cancel.Click += new System.EventHandler(this.btn_cancel_Click);
            // 
            // txt_x
            // 
            this.txt_x.Font = new System.Drawing.Font("Microsoft Sans Serif", 19.8F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.txt_x.Location = new System.Drawing.Point(922, 271);
            this.txt_x.Margin = new System.Windows.Forms.Padding(2, 2, 2, 2);
            this.txt_x.Name = "txt_x";
            this.txt_x.Size = new System.Drawing.Size(176, 45);
            this.txt_x.TabIndex = 43;
            this.txt_x.Text = "0";
            this.txt_x.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // txt_y
            // 
            this.txt_y.Font = new System.Drawing.Font("Microsoft Sans Serif", 19.8F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.txt_y.Location = new System.Drawing.Point(922, 335);
            this.txt_y.Margin = new System.Windows.Forms.Padding(2, 2, 2, 2);
            this.txt_y.Name = "txt_y";
            this.txt_y.Size = new System.Drawing.Size(176, 45);
            this.txt_y.TabIndex = 44;
            this.txt_y.Text = "0";
            this.txt_y.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // txt_theta
            // 
            this.txt_theta.Font = new System.Drawing.Font("Microsoft Sans Serif", 19.8F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.txt_theta.Location = new System.Drawing.Point(922, 398);
            this.txt_theta.Margin = new System.Windows.Forms.Padding(2, 2, 2, 2);
            this.txt_theta.Name = "txt_theta";
            this.txt_theta.Size = new System.Drawing.Size(176, 45);
            this.txt_theta.TabIndex = 45;
            this.txt_theta.Text = "0";
            this.txt_theta.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(922, 254);
            this.label1.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(21, 15);
            this.label1.TabIndex = 46;
            this.label1.Text = "X :";
            // 
            // label13
            // 
            this.label13.AutoSize = true;
            this.label13.Location = new System.Drawing.Point(922, 318);
            this.label13.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label13.Name = "label13";
            this.label13.Size = new System.Drawing.Size(20, 15);
            this.label13.TabIndex = 47;
            this.label13.Text = "Y :";
            // 
            // label14
            // 
            this.label14.AutoSize = true;
            this.label14.Location = new System.Drawing.Point(922, 382);
            this.label14.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label14.Name = "label14";
            this.label14.Size = new System.Drawing.Size(44, 15);
            this.label14.TabIndex = 48;
            this.label14.Text = "Theta :";
            // 
            // main_timer
            // 
            this.main_timer.Enabled = true;
            this.main_timer.Interval = 1000;
            this.main_timer.Tick += new System.EventHandler(this.main_timer_Tick);
            // 
            // rad_laser
            // 
            this.rad_laser.AutoSize = true;
            this.rad_laser.Checked = true;
            this.rad_laser.Location = new System.Drawing.Point(340, 6);
            this.rad_laser.Margin = new System.Windows.Forms.Padding(2, 2, 2, 2);
            this.rad_laser.Name = "rad_laser";
            this.rad_laser.Size = new System.Drawing.Size(67, 19);
            this.rad_laser.TabIndex = 51;
            this.rad_laser.TabStop = true;
            this.rad_laser.Text = "LASER";
            this.rad_laser.UseVisualStyleBackColor = true;
            // 
            // btn_reset_map
            // 
            this.btn_reset_map.Location = new System.Drawing.Point(902, 214);
            this.btn_reset_map.Margin = new System.Windows.Forms.Padding(2, 2, 2, 2);
            this.btn_reset_map.Name = "btn_reset_map";
            this.btn_reset_map.Size = new System.Drawing.Size(214, 33);
            this.btn_reset_map.TabIndex = 54;
            this.btn_reset_map.Text = "reset map";
            this.btn_reset_map.UseVisualStyleBackColor = true;
            // 
            // label12
            // 
            this.label12.AutoSize = true;
            this.label12.Location = new System.Drawing.Point(940, 7);
            this.label12.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label12.Name = "label12";
            this.label12.Size = new System.Drawing.Size(63, 15);
            this.label12.TabIndex = 56;
            this.label12.Text = "Robot_ip :";
            // 
            // txt_robot_ip
            // 
            this.txt_robot_ip.Font = new System.Drawing.Font("Microsoft Sans Serif", 19.8F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.txt_robot_ip.Location = new System.Drawing.Point(940, 24);
            this.txt_robot_ip.Margin = new System.Windows.Forms.Padding(2, 2, 2, 2);
            this.txt_robot_ip.Name = "txt_robot_ip";
            this.txt_robot_ip.Size = new System.Drawing.Size(176, 45);
            this.txt_robot_ip.TabIndex = 55;
            this.txt_robot_ip.Text = "192.168.1.7";
            this.txt_robot_ip.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // label15
            // 
            this.label15.AutoSize = true;
            this.label15.Location = new System.Drawing.Point(940, 63);
            this.label15.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label15.Name = "label15";
            this.label15.Size = new System.Drawing.Size(68, 15);
            this.label15.TabIndex = 58;
            this.label15.Text = "Matlab_ip :";
            // 
            // txt_matlab_ip
            // 
            this.txt_matlab_ip.Font = new System.Drawing.Font("Microsoft Sans Serif", 19.8F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.txt_matlab_ip.Location = new System.Drawing.Point(940, 79);
            this.txt_matlab_ip.Margin = new System.Windows.Forms.Padding(2, 2, 2, 2);
            this.txt_matlab_ip.Name = "txt_matlab_ip";
            this.txt_matlab_ip.Size = new System.Drawing.Size(176, 45);
            this.txt_matlab_ip.TabIndex = 57;
            this.txt_matlab_ip.Text = "127.0.0.1";
            this.txt_matlab_ip.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // btn_config
            // 
            this.btn_config.Location = new System.Drawing.Point(902, 126);
            this.btn_config.Margin = new System.Windows.Forms.Padding(2, 2, 2, 2);
            this.btn_config.Name = "btn_config";
            this.btn_config.Size = new System.Drawing.Size(214, 33);
            this.btn_config.TabIndex = 59;
            this.btn_config.Text = "save config";
            this.btn_config.UseVisualStyleBackColor = true;
            this.btn_config.Click += new System.EventHandler(this.btn_config_Click);
            // 
            // label16
            // 
            this.label16.AutoSize = true;
            this.label16.Location = new System.Drawing.Point(9, 340);
            this.label16.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label16.Name = "label16";
            this.label16.Size = new System.Drawing.Size(100, 15);
            this.label16.TabIndex = 61;
            this.label16.Text = "Current Position :";
            // 
            // txt_current_position
            // 
            this.txt_current_position.Font = new System.Drawing.Font("Microsoft Sans Serif", 19.8F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.txt_current_position.Location = new System.Drawing.Point(11, 356);
            this.txt_current_position.Margin = new System.Windows.Forms.Padding(2, 2, 2, 2);
            this.txt_current_position.Name = "txt_current_position";
            this.txt_current_position.ReadOnly = true;
            this.txt_current_position.Size = new System.Drawing.Size(278, 45);
            this.txt_current_position.TabIndex = 60;
            this.txt_current_position.Text = "0";
            this.txt_current_position.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // label17
            // 
            this.label17.AutoSize = true;
            this.label17.Location = new System.Drawing.Point(9, 392);
            this.label17.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label17.Name = "label17";
            this.label17.Size = new System.Drawing.Size(129, 15);
            this.label17.TabIndex = 63;
            this.label17.Text = "Current Goal Poistion :";
            // 
            // txt_current_goal_position
            // 
            this.txt_current_goal_position.Font = new System.Drawing.Font("Microsoft Sans Serif", 19.8F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.txt_current_goal_position.Location = new System.Drawing.Point(11, 409);
            this.txt_current_goal_position.Margin = new System.Windows.Forms.Padding(2, 2, 2, 2);
            this.txt_current_goal_position.Name = "txt_current_goal_position";
            this.txt_current_goal_position.ReadOnly = true;
            this.txt_current_goal_position.Size = new System.Drawing.Size(278, 45);
            this.txt_current_goal_position.TabIndex = 62;
            this.txt_current_goal_position.Text = "0";
            this.txt_current_goal_position.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // btn_reset2
            // 
            this.btn_reset2.Location = new System.Drawing.Point(11, 506);
            this.btn_reset2.Margin = new System.Windows.Forms.Padding(2, 2, 2, 2);
            this.btn_reset2.Name = "btn_reset2";
            this.btn_reset2.Size = new System.Drawing.Size(66, 33);
            this.btn_reset2.TabIndex = 64;
            this.btn_reset2.Text = "Reset";
            this.btn_reset2.UseVisualStyleBackColor = true;
            this.btn_reset2.Click += new System.EventHandler(this.btn_reset2_Click);
            // 
            // btn_reset3
            // 
            this.btn_reset3.Location = new System.Drawing.Point(11, 554);
            this.btn_reset3.Margin = new System.Windows.Forms.Padding(2, 2, 2, 2);
            this.btn_reset3.Name = "btn_reset3";
            this.btn_reset3.Size = new System.Drawing.Size(66, 33);
            this.btn_reset3.TabIndex = 65;
            this.btn_reset3.Text = "Reset";
            this.btn_reset3.UseVisualStyleBackColor = true;
            this.btn_reset3.Click += new System.EventHandler(this.btn_reset3_Click);
            // 
            // btn_stop
            // 
            this.btn_stop.Location = new System.Drawing.Point(688, 454);
            this.btn_stop.Margin = new System.Windows.Forms.Padding(2, 2, 2, 2);
            this.btn_stop.Name = "btn_stop";
            this.btn_stop.Size = new System.Drawing.Size(206, 148);
            this.btn_stop.TabIndex = 66;
            this.btn_stop.Text = "STOP";
            this.btn_stop.UseVisualStyleBackColor = true;
            this.btn_stop.Click += new System.EventHandler(this.btn_stop_Click);
            // 
            // map_timer
            // 
            this.map_timer.Enabled = true;
            this.map_timer.Tick += new System.EventHandler(this.map_timer_Tick);
            // 
            // nav_timer
            // 
            this.nav_timer.Interval = 50;
            // 
            // matlab
            // 
            this.matlab.Enabled = true;
            this.matlab.Tick += new System.EventHandler(this.matlab_Tick);
            // 
            // frmMain
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(1120, 609);
            this.Controls.Add(this.btn_stop);
            this.Controls.Add(this.btn_reset3);
            this.Controls.Add(this.btn_reset2);
            this.Controls.Add(this.label17);
            this.Controls.Add(this.txt_current_goal_position);
            this.Controls.Add(this.label16);
            this.Controls.Add(this.txt_current_position);
            this.Controls.Add(this.btn_config);
            this.Controls.Add(this.label15);
            this.Controls.Add(this.txt_matlab_ip);
            this.Controls.Add(this.label12);
            this.Controls.Add(this.txt_robot_ip);
            this.Controls.Add(this.btn_reset_map);
            this.Controls.Add(this.rad_laser);
            this.Controls.Add(this.label14);
            this.Controls.Add(this.label13);
            this.Controls.Add(this.label1);
            this.Controls.Add(this.txt_theta);
            this.Controls.Add(this.txt_y);
            this.Controls.Add(this.txt_x);
            this.Controls.Add(this.btn_cancel);
            this.Controls.Add(this.btn_navigation);
            this.Controls.Add(this.btn_set_target);
            this.Controls.Add(this.panel_matlab);
            this.Controls.Add(this.panel_robot);
            this.Controls.Add(this.track_speed_1);
            this.Controls.Add(this.label10);
            this.Controls.Add(this.label9);
            this.Controls.Add(this.lst_points);
            this.Controls.Add(this.track_position);
            this.Controls.Add(this.track_speed_2);
            this.Controls.Add(this.btn_reset1);
            this.Controls.Add(this.label8);
            this.Controls.Add(this.txt_alarm);
            this.Controls.Add(this.label7);
            this.Controls.Add(this.txt_sensor);
            this.Controls.Add(this.label6);
            this.Controls.Add(this.txt_voltage);
            this.Controls.Add(this.btn_led);
            this.Controls.Add(this.txt_set_position);
            this.Controls.Add(this.txt_set_speed_2);
            this.Controls.Add(this.txt_set_speed_1);
            this.Controls.Add(this.Load);
            this.Controls.Add(this.txt_load);
            this.Controls.Add(this.label4);
            this.Controls.Add(this.txt_position);
            this.Controls.Add(this.label3);
            this.Controls.Add(this.txt_speed2);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.txt_speed1);
            this.Controls.Add(this.img_map);
            this.FormBorderStyle = System.Windows.Forms.FormBorderStyle.FixedToolWindow;
            this.Margin = new System.Windows.Forms.Padding(2, 2, 2, 2);
            this.Name = "frmMain";
            this.StartPosition = System.Windows.Forms.FormStartPosition.CenterScreen;
            this.Text = "Main V 1.0.0";
            this.FormClosed += new System.Windows.Forms.FormClosedEventHandler(this.frmMain_FormClosed);
            ((System.ComponentModel.ISupportInitialize)(this.img_map)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.track_speed_2)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.track_position)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.track_speed_1)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.PictureBox img_map;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.TextBox txt_speed1;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.TextBox txt_speed2;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.TextBox txt_position;
        private System.Windows.Forms.Label Load;
        private System.Windows.Forms.TextBox txt_load;
        private System.Windows.Forms.TextBox txt_set_speed_1;
        private System.Windows.Forms.TextBox txt_set_speed_2;
        private System.Windows.Forms.TextBox txt_set_position;
        private System.Windows.Forms.Button btn_led;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.TextBox txt_voltage;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.TextBox txt_sensor;
        private System.Windows.Forms.Label label8;
        private System.Windows.Forms.TextBox txt_alarm;
        private System.Windows.Forms.Button btn_reset1;
        private System.Windows.Forms.TrackBar track_speed_2;
        private System.Windows.Forms.TrackBar track_position;
        private System.Windows.Forms.ListBox lst_points;
        private System.Windows.Forms.Label label9;
        private System.Windows.Forms.Label label10;
        private System.Windows.Forms.TrackBar track_speed_1;
        private System.Windows.Forms.Panel panel_robot;
        private System.Windows.Forms.Panel panel_matlab;
        private System.Windows.Forms.Button btn_set_target;
        private System.Windows.Forms.Button btn_navigation;
        private System.Windows.Forms.Button btn_cancel;
        private System.Windows.Forms.TextBox txt_x;
        private System.Windows.Forms.TextBox txt_y;
        private System.Windows.Forms.TextBox txt_theta;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Label label13;
        private System.Windows.Forms.Label label14;
        private System.Windows.Forms.Timer main_timer;
        private System.Windows.Forms.RadioButton rad_laser;
        private System.Windows.Forms.Button btn_reset_map;
        private System.Windows.Forms.Label label12;
        private System.Windows.Forms.TextBox txt_robot_ip;
        private System.Windows.Forms.Label label15;
        private System.Windows.Forms.TextBox txt_matlab_ip;
        private System.Windows.Forms.Button btn_config;
        private System.Windows.Forms.Label label16;
        private System.Windows.Forms.TextBox txt_current_position;
        private System.Windows.Forms.Label label17;
        private System.Windows.Forms.TextBox txt_current_goal_position;
        private System.Windows.Forms.Button btn_reset2;
        private System.Windows.Forms.Button btn_reset3;
        private System.Windows.Forms.Button btn_stop;
        private System.Windows.Forms.Timer map_timer;
        private System.Windows.Forms.Timer nav_timer;
        private System.Windows.Forms.Timer matlab;
    }
}

