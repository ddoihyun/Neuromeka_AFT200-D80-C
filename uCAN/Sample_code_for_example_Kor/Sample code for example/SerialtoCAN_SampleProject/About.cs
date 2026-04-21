using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;

namespace SerialtoCAN_SampleProject
{
    public partial class About : Form
    {
        //const string VIRSION = "V1.0.0";    //Release
        const string VIRSION = "V1.0.1";    //MakeReceivePacket() bug fix


        public About()
        {
            InitializeComponent();
        }

        private void About_Load(object sender, EventArgs e)
        {
            VersionLabel.Text = "Serial to sCAN/uCAN " + VIRSION;
        }
    }
}
