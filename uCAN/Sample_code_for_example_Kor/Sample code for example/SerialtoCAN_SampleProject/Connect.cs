using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Windows.Forms;

namespace SerialtoCAN_SampleProject
{
    public partial class Connect : Form
    {
        private string[] sendValue = new string[6];


        public Connect()
        {
            InitializeComponent();
        }

        private void Connect_Load(object sender, EventArgs e)
        {
            comPortCBox.DataSource = SerialPort.GetPortNames();
            comPortCBox.SelectedIndex = 0;

            String[] baudRateText = { "300", "600", "1200", "2400", "4800", "9600", "14400", "19200", "28800", "38400", "57600", "115200", "460800", "921600" };
            baudRateCBox.Items.AddRange(baudRateText);
            baudRateCBox.SelectedIndex = 11;

            String[] dataBitsText = { "1", "2", "3", "4", "5", "6", "7", "8" };
            dataBitsCBox.Items.AddRange(dataBitsText);
            dataBitsCBox.SelectedIndex = 7;

            String[] parityText = { "None", "odd", "Even", "Mark", "Space" };
            parityCBox.Items.AddRange(parityText);
            parityCBox.SelectedIndex = 0;

            String[] stopBitsText = { "None", "One", "Two", "OnePointFive" };
            stopBitsCBox.Items.AddRange(stopBitsText);
            stopBitsCBox.SelectedIndex = 1;

            String[] flowControlText = { "None", "SW", "HW", "SW/HW" };
            flowControlCBox.Items.AddRange(flowControlText);
            flowControlCBox.SelectedIndex = 0;
        }

        private void connectButton_Click(object sender, EventArgs e)
        {
            DialogResult = System.Windows.Forms.DialogResult.OK;
        }

        public string[] PassValue
        {
            get
            {
                sendValue[0] = comPortCBox.Text;
                sendValue[1] = baudRateCBox.Text;
                sendValue[2] = dataBitsCBox.Text;
                sendValue[3] = parityCBox.SelectedIndex + "";
                sendValue[4] = stopBitsCBox.Text;
                sendValue[5] = flowControlCBox.SelectedIndex + "";

                return sendValue;
            }
        }
    }
}
