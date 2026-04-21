using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Text.RegularExpressions;
using System.Windows.Forms;

namespace SerialtoCAN_SampleProject
{
    public partial class SerialtoCAN_Sample : Form
    {
        SerialPort serialPort;
        List<TextBox> TextBoxList;
        List<byte> RecvDataList;
        byte[] receiveData;

        delegate void AppendTextDelegate(RichTextBox ctrl, string s);
        static AppendTextDelegate _textAppender;

        //t=0x74, T=0x54, e=0x65, E=0x45

        const byte STD_DATA = 0x74;
        const byte STD_REMOTE = 0x54;
        const byte EXT_DATA = 0x65;
        const byte EXT_REMOTE = 0x45;

        const int PACKET_CR_LENGTH = 1;
        const int PACKET_STD_HEADER_LENGTH = 5;
        const int PACKET_EXT_HEADER_LENGTH = 10;

        const int PACKET_FORMAT = 0;
        const int PACKET_STD_DLC = 4;
        const int PACKET_EXT_DLC = 9;

        public SerialtoCAN_Sample()
        {
            InitializeComponent();
        }

        private void SerialtoCAN_Sample_Load(object sender, EventArgs e)
        {
            serialPort = new SerialPort();
            serialPort.DataReceived += SerialPort_DataReceived;

            _textAppender = new AppendTextDelegate(AppendText);

            sendButton.Enabled = false;

            String[] formatComboBoxText = { "STD DATA","STD REMOTE","EXT DATA","EXT REMOTE" };
            formatComboBox.Items.AddRange(formatComboBoxText);
            formatComboBox.SelectedIndex = 0;

            String[] dlcComboBoxText = { "0", "1", "2", "3", "4", "5", "6", "7", "8" };
            dlcComboBox.Items.AddRange(dlcComboBoxText);
            dlcComboBox.SelectedIndex = 8;

            TextBoxList = new List<TextBox>();
            TextBoxList.Add(dataTextBox1);
            TextBoxList.Add(dataTextBox2);
            TextBoxList.Add(dataTextBox3);
            TextBoxList.Add(dataTextBox4);
            TextBoxList.Add(dataTextBox5);
            TextBoxList.Add(dataTextBox6);
            TextBoxList.Add(dataTextBox7);
            TextBoxList.Add(dataTextBox8);
        }

        public void SerialPort_DataReceived(object sender, System.IO.Ports.SerialDataReceivedEventArgs e)
        {
            try
            {
                if (serialPort.IsOpen)
                {
                    RecvDataList = new List<byte>();
                    byte[] recvData;
                    while (serialPort.BytesToRead != 0)
                    {
                        RecvDataList.Add((byte)serialPort.ReadByte());
                    }

                    recvData = RecvDataList.ToArray();

                    if (receiveData == null)
                    {
                        receiveData = recvData;
                    }
                    else
                    {
                        byte[] tempReceiveData = receiveData;
                        receiveData = new byte[tempReceiveData.Length + recvData.Length];
                        Array.Copy(tempReceiveData, 0, receiveData, 0, tempReceiveData.Length);
                        Array.Copy(recvData, 0, receiveData, tempReceiveData.Length, recvData.Length);
                    }
                    MakeReceivePacket();
                }
            }
            catch (Exception ex)
            {
                AppendText(monitorRichTextBox, "DataReceived Failed - " + ex);
            }
        }

        ///EXAMPLE
        ///0 123 4 56 78 910 1112 1314 1516 1718 1920 21
        ///t 001 8 11 22 33  4 4  5 5  6 6  7 7  8 8  0d
        ///0 12345678 9 1011 1213 1415 1617 1819 2021 2223 2425 26
        ///e 00000123 8 1 1  2 2  3 3  4 4  5 5  6 6  7 7  8 8  0d
        ///t=0x74, T=0x54, e=0x65, E=0x45
        public void MakeReceivePacket()
        {
            int dataLength;
            if (!(receiveData[PACKET_FORMAT] == STD_DATA || receiveData[PACKET_FORMAT] == STD_REMOTE ||
                receiveData[PACKET_FORMAT] == EXT_DATA || receiveData[PACKET_FORMAT] == EXT_REMOTE))
            {
                receiveData = null;
            }
            else if (receiveData[PACKET_FORMAT] == STD_DATA && receiveData.Length >= PACKET_STD_HEADER_LENGTH)
            {
                dataLength = PACKET_STD_HEADER_LENGTH + ((receiveData[PACKET_STD_DLC] - 0x30) * 2) + PACKET_CR_LENGTH;
                if (receiveData.Length == dataLength)
                {
                    AppendText(monitorRichTextBox, Encoding.ASCII.GetString(receiveData));                              //received std data
                    receiveData = null;
                }
                else if (receiveData.Length > dataLength)
                {
                    byte[] tempReceiveData = new byte[dataLength];
                    Array.Copy(receiveData, 0, tempReceiveData, 0, dataLength);
                    AppendText(monitorRichTextBox, Encoding.ASCII.GetString(tempReceiveData));                          //received std data
                    tempReceiveData = receiveData;
                    receiveData = new byte[tempReceiveData.Length - dataLength];
                    Array.Copy(tempReceiveData, dataLength, receiveData, 0, receiveData.Length);
                    MakeReceivePacket();
                }
            }
            else if (receiveData[PACKET_FORMAT] == STD_REMOTE && receiveData.Length >= PACKET_STD_HEADER_LENGTH)
            {
                dataLength = PACKET_STD_HEADER_LENGTH + PACKET_CR_LENGTH;
                if (receiveData.Length == dataLength)
                {
                    AppendText(monitorRichTextBox, Encoding.ASCII.GetString(receiveData));                              //received std remote
                    receiveData = null;
                }
                else if (receiveData.Length > dataLength)
                {
                    byte[] tempReceiveData = new byte[dataLength];
                    Array.Copy(receiveData, 0, tempReceiveData, 0, dataLength);
                    AppendText(monitorRichTextBox, Encoding.ASCII.GetString(tempReceiveData));                          //received std remote
                    tempReceiveData = receiveData;
                    receiveData = new byte[tempReceiveData.Length - dataLength];
                    Array.Copy(tempReceiveData, dataLength, receiveData, 0, receiveData.Length);
                    MakeReceivePacket();
                }
            }
            else if (receiveData[PACKET_FORMAT] == EXT_DATA && receiveData.Length >= PACKET_EXT_HEADER_LENGTH)
            {
                dataLength = PACKET_EXT_HEADER_LENGTH + ((receiveData[PACKET_EXT_DLC] - 0x30) * 2) + PACKET_CR_LENGTH;

                if (receiveData.Length == dataLength)
                {
                    AppendText(monitorRichTextBox, Encoding.ASCII.GetString(receiveData));                              //received ext data
                    receiveData = null;
                }
                else if (receiveData.Length > dataLength)
                {
                    byte[] tempReceiveData = new byte[dataLength];
                    Array.Copy(receiveData, 0, tempReceiveData, 0, dataLength);
                    AppendText(monitorRichTextBox, Encoding.ASCII.GetString(tempReceiveData));                          //received ext data
                    tempReceiveData = receiveData;
                    receiveData = new byte[tempReceiveData.Length - dataLength];
                    Array.Copy(tempReceiveData, dataLength, receiveData, 0, receiveData.Length);
                    MakeReceivePacket();
                }
            }
            else if (receiveData[PACKET_FORMAT] == EXT_REMOTE && receiveData.Length >= PACKET_EXT_HEADER_LENGTH)
            {
                dataLength = PACKET_EXT_HEADER_LENGTH + PACKET_CR_LENGTH;
                if (receiveData.Length == dataLength)
                {
                    AppendText(monitorRichTextBox, Encoding.ASCII.GetString(receiveData));                              //received ext remote
                    receiveData = null;
                }
                else if (receiveData.Length > dataLength)
                {
                    byte[] tempReceiveData = new byte[dataLength];
                    Array.Copy(receiveData, 0, tempReceiveData, 0, dataLength);
                    AppendText(monitorRichTextBox, Encoding.ASCII.GetString(tempReceiveData));                          //received ext remote
                    tempReceiveData = receiveData;
                    receiveData = new byte[tempReceiveData.Length - dataLength];
                    Array.Copy(tempReceiveData, dataLength, receiveData, 0, receiveData.Length);
                    MakeReceivePacket();
                }
            }
        }

        private void sendButton_Click(object sender, EventArgs e)
        {
            byte[] sendData = MakeSendPacket();

            try
            {
                serialPort.Write(sendData, 0, sendData.Length);
            }
            catch (Exception ex)
            {
                Console.WriteLine(ex + "");
            }
        }

        public byte[] MakeSendPacket()
        {
            string sendData = "";
            if (formatComboBox.SelectedIndex == 0)                                                  //make std data
            {
                sendData = ((char)STD_DATA).ToString();

                for (int i = 3; i > idTextBox.TextLength; i--)
                {
                    sendData += "0";
                }
                sendData += idTextBox.Text;
                sendData += dlcComboBox.SelectedIndex;

                for (int i = 0; i < dlcComboBox.SelectedIndex; i++)
                {
                    for (int j = 2; j > TextBoxList[i].TextLength; j--)
                    {
                        sendData += "0";
                    }
                    sendData += TextBoxList[i].Text;
                }
            }
            else if (formatComboBox.SelectedIndex == 1)                                             //make std remote
            {
                sendData = ((char)STD_REMOTE).ToString();

                for (int i = 3; i > idTextBox.TextLength; i--)
                {
                    sendData += "0";
                }
                sendData += idTextBox.Text;
                sendData += dlcComboBox.SelectedIndex;
            }
            else if (formatComboBox.SelectedIndex == 2)                                             //make ext data
            {
                sendData = ((char)EXT_DATA).ToString();

                for (int i = 8; i > idTextBox.TextLength; i--)
                {
                    sendData += "0";
                }
                sendData += idTextBox.Text;
                sendData += dlcComboBox.SelectedIndex;

                for (int i = 0; i < dlcComboBox.SelectedIndex; i++)
                {
                    for (int j = 2; j > TextBoxList[i].TextLength; j--)
                    {
                        sendData += "0";
                    }
                    sendData += TextBoxList[i].Text;
                }
            }
            else                                                                                    //make ext remote
            {
                sendData = ((char)EXT_REMOTE).ToString();

                for (int i = 8; i > idTextBox.TextLength; i--)
                {
                    sendData += "0";
                }
                sendData += idTextBox.Text;
                sendData += dlcComboBox.SelectedIndex;
            }
            sendData += "\r";

            return Encoding.ASCII.GetBytes(sendData);
        }

        private void connectToolStripMenuItem_Click(object sender, EventArgs e)
        {
            Connect connect = new Connect();
            Point parentPoint = this.Location;
            connect.StartPosition = FormStartPosition.Manual;
            connect.Location = new Point(parentPoint.X + 50, parentPoint.Y + 50);
            if (DialogResult.OK == connect.ShowDialog())
            {
                string portName, stopBits;
                int baudRate, dataBits, parity, handshake;

                portName = connect.PassValue[0];
                baudRate = int.Parse(connect.PassValue[1]);
                dataBits = int.Parse(connect.PassValue[2]);
                parity = int.Parse(connect.PassValue[3]);
                stopBits = connect.PassValue[4];
                handshake = int.Parse(connect.PassValue[5]);

                try
                {
                    serialPort.PortName = portName;
                    serialPort.BaudRate = baudRate;
                    serialPort.DataBits = dataBits;
                    serialPort.Parity = (Parity)parity;
                    serialPort.StopBits = (StopBits)Enum.Parse(typeof(StopBits), stopBits, true);
                    serialPort.Handshake = (Handshake)handshake;

                    serialPort.Open();
                    connectToolStripMenuItem.Enabled = false;
                    sendButton.Enabled = true;
                    this.Text = "SerialtoCAN_Sample (" + portName + ")";
                }
                catch (Exception ex)
                {
                    AppendText(monitorRichTextBox, "Connect Failed - " + ex);
                    AppendText(monitorRichTextBox, Environment.NewLine);
                }
            }
        }

        private void disconnectToolStripMenuItem_Click(object sender, EventArgs e)
        {
            try
            {
                serialPort.Close();
                AppendText(monitorRichTextBox, "SerialPort Closed OK");
                connectToolStripMenuItem.Enabled = true;
                sendButton.Enabled = false;
            }
            catch (Exception ex)
            {
                AppendText(monitorRichTextBox, "SerialPort Close Failed - " + ex);
                AppendText(monitorRichTextBox, Environment.NewLine);
            }

        }

        private void aboutToolStripMenuItem_Click(object sender, EventArgs e)
        {
            About aboutForm = new About();
            Point parentPoint = this.Location;
            aboutForm.StartPosition = FormStartPosition.Manual;
            aboutForm.Location = new Point(parentPoint.X + 50, parentPoint.Y + 50);
            aboutForm.ShowDialog();
        }


        private void formatComboBox_SelectedIndexChanged(object sender, EventArgs e)
        {
            if (formatComboBox.SelectedIndex < 2)
            {
                idTextBox.MaxLength = 3;
            }
            else
            {
                idTextBox.MaxLength = 8;
            }
        }
                     
        public void AppendText(RichTextBox ctrl, string s)
        {
            if (ctrl.InvokeRequired)
            {
                ctrl.Invoke(_textAppender, ctrl, s);
            }
            else
            {
                String now = System.DateTime.Now.ToString("HH:mm:ss::fff");

                ctrl.AppendText(now + " : ");
                ctrl.AppendText(s);
                ctrl.ScrollToCaret();

            }
        }

        private void dataTextBox_TextChanged(object sender, EventArgs e)
        {
            TextBox textBox = (TextBox)sender;

            try
            {
                if (textBox.Text == "")
                {
                    return;
                }
                if (Convert.ToInt32(textBox.Text, 16) > 255)
                {
                    MessageBoxEx.Show(this, "Can not write more than FF", "ERROR");
                    textBox.Text = "00";
                }
            }
            catch
            {
                MessageBoxEx.Show(this, "Please enter valid data", "ERROR");
                textBox.Text = "00";
            }
        }


        private void tBox_KeyPress(object sender, KeyPressEventArgs e)
        {
            Regex reg = new Regex(@"^[a-fA-F0-9]+$");

            if (!(reg.IsMatch(e.KeyChar.ToString()) || e.KeyChar == Convert.ToChar(Keys.Back)))
            {
                e.Handled = true;
            }
        }

        private void idTextBox_TextChanged(object sender, EventArgs e)
        {
            try
            {
                if (idTextBox.Text == "")
                {
                    return;
                }
                if(formatComboBox.SelectedIndex < 2)
                {
                    if (Convert.ToUInt32(idTextBox.Text, 16) > 0x7FF)
                    {
                        MessageBoxEx.Show(this, "Can not write more than 7FF", "ERROR");
                        idTextBox.Text = "123";
                    }
                }
                else
                {
                    if (Convert.ToUInt64(idTextBox.Text, 16) > (long)0x1FFFFFFF)
                    {
                        MessageBoxEx.Show(this, "Can not write more than 1FFFFFFF", "ERROR");
                        idTextBox.Text = "123";
                    }
                }

            }
            catch
            {
                MessageBoxEx.Show(this, "Please enter valid data", "ERROR");
                idTextBox.Text = "123";
            }
        }
    }
}
