namespace SerialtoCAN_SampleProject
{
    partial class SerialtoCAN_Sample
    {
        /// <summary>
        /// 필수 디자이너 변수입니다.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// 사용 중인 모든 리소스를 정리합니다.
        /// </summary>
        /// <param name="disposing">관리되는 리소스를 삭제해야 하면 true이고, 그렇지 않으면 false입니다.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form 디자이너에서 생성한 코드

        /// <summary>
        /// 디자이너 지원에 필요한 메서드입니다. 
        /// 이 메서드의 내용을 코드 편집기로 수정하지 마세요.
        /// </summary>
        private void InitializeComponent()
        {
            this.menuStrip1 = new System.Windows.Forms.MenuStrip();
            this.actionToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.connectToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.disconnectToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.helpToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.aboutToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.formatComboBox = new System.Windows.Forms.ComboBox();
            this.dlcComboBox = new System.Windows.Forms.ComboBox();
            this.idTextBox = new System.Windows.Forms.TextBox();
            this.dataTextBox1 = new System.Windows.Forms.TextBox();
            this.dataTextBox2 = new System.Windows.Forms.TextBox();
            this.dataTextBox3 = new System.Windows.Forms.TextBox();
            this.dataTextBox4 = new System.Windows.Forms.TextBox();
            this.dataTextBox5 = new System.Windows.Forms.TextBox();
            this.dataTextBox6 = new System.Windows.Forms.TextBox();
            this.dataTextBox7 = new System.Windows.Forms.TextBox();
            this.dataTextBox8 = new System.Windows.Forms.TextBox();
            this.label1 = new System.Windows.Forms.Label();
            this.label2 = new System.Windows.Forms.Label();
            this.label3 = new System.Windows.Forms.Label();
            this.label4 = new System.Windows.Forms.Label();
            this.sendButton = new System.Windows.Forms.Button();
            this.monitorRichTextBox = new System.Windows.Forms.RichTextBox();
            this.menuStrip1.SuspendLayout();
            this.SuspendLayout();
            // 
            // menuStrip1
            // 
            this.menuStrip1.Items.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.actionToolStripMenuItem,
            this.helpToolStripMenuItem});
            this.menuStrip1.Location = new System.Drawing.Point(0, 0);
            this.menuStrip1.Name = "menuStrip1";
            this.menuStrip1.Size = new System.Drawing.Size(443, 24);
            this.menuStrip1.TabIndex = 0;
            this.menuStrip1.Text = "menuStrip1";
            // 
            // actionToolStripMenuItem
            // 
            this.actionToolStripMenuItem.DropDownItems.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.connectToolStripMenuItem,
            this.disconnectToolStripMenuItem});
            this.actionToolStripMenuItem.Name = "actionToolStripMenuItem";
            this.actionToolStripMenuItem.Size = new System.Drawing.Size(54, 20);
            this.actionToolStripMenuItem.Text = "Action";
            // 
            // connectToolStripMenuItem
            // 
            this.connectToolStripMenuItem.Name = "connectToolStripMenuItem";
            this.connectToolStripMenuItem.Size = new System.Drawing.Size(134, 22);
            this.connectToolStripMenuItem.Text = "Connect";
            this.connectToolStripMenuItem.Click += new System.EventHandler(this.connectToolStripMenuItem_Click);
            // 
            // disconnectToolStripMenuItem
            // 
            this.disconnectToolStripMenuItem.Name = "disconnectToolStripMenuItem";
            this.disconnectToolStripMenuItem.Size = new System.Drawing.Size(134, 22);
            this.disconnectToolStripMenuItem.Text = "Disconnect";
            this.disconnectToolStripMenuItem.Click += new System.EventHandler(this.disconnectToolStripMenuItem_Click);
            // 
            // helpToolStripMenuItem
            // 
            this.helpToolStripMenuItem.DropDownItems.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.aboutToolStripMenuItem});
            this.helpToolStripMenuItem.Name = "helpToolStripMenuItem";
            this.helpToolStripMenuItem.Size = new System.Drawing.Size(44, 20);
            this.helpToolStripMenuItem.Text = "Help";
            // 
            // aboutToolStripMenuItem
            // 
            this.aboutToolStripMenuItem.Name = "aboutToolStripMenuItem";
            this.aboutToolStripMenuItem.Size = new System.Drawing.Size(107, 22);
            this.aboutToolStripMenuItem.Text = "About";
            this.aboutToolStripMenuItem.Click += new System.EventHandler(this.aboutToolStripMenuItem_Click);
            // 
            // formatComboBox
            // 
            this.formatComboBox.FormattingEnabled = true;
            this.formatComboBox.Location = new System.Drawing.Point(12, 52);
            this.formatComboBox.Name = "formatComboBox";
            this.formatComboBox.Size = new System.Drawing.Size(146, 20);
            this.formatComboBox.TabIndex = 1;
            this.formatComboBox.SelectedIndexChanged += new System.EventHandler(this.formatComboBox_SelectedIndexChanged);
            // 
            // dlcComboBox
            // 
            this.dlcComboBox.FormattingEnabled = true;
            this.dlcComboBox.Location = new System.Drawing.Point(290, 51);
            this.dlcComboBox.Name = "dlcComboBox";
            this.dlcComboBox.Size = new System.Drawing.Size(44, 20);
            this.dlcComboBox.TabIndex = 2;
            // 
            // idTextBox
            // 
            this.idTextBox.Location = new System.Drawing.Point(192, 51);
            this.idTextBox.Name = "idTextBox";
            this.idTextBox.Size = new System.Drawing.Size(70, 21);
            this.idTextBox.TabIndex = 3;
            this.idTextBox.Text = "123";
            this.idTextBox.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            this.idTextBox.TextChanged += new System.EventHandler(this.idTextBox_TextChanged);
            this.idTextBox.KeyPress += new System.Windows.Forms.KeyPressEventHandler(this.tBox_KeyPress);
            // 
            // dataTextBox1
            // 
            this.dataTextBox1.Location = new System.Drawing.Point(12, 98);
            this.dataTextBox1.MaxLength = 2;
            this.dataTextBox1.Name = "dataTextBox1";
            this.dataTextBox1.Size = new System.Drawing.Size(35, 21);
            this.dataTextBox1.TabIndex = 4;
            this.dataTextBox1.Text = "11";
            this.dataTextBox1.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            this.dataTextBox1.TextChanged += new System.EventHandler(this.dataTextBox_TextChanged);
            this.dataTextBox1.KeyPress += new System.Windows.Forms.KeyPressEventHandler(this.tBox_KeyPress);
            // 
            // dataTextBox2
            // 
            this.dataTextBox2.Location = new System.Drawing.Point(53, 98);
            this.dataTextBox2.MaxLength = 2;
            this.dataTextBox2.Name = "dataTextBox2";
            this.dataTextBox2.Size = new System.Drawing.Size(35, 21);
            this.dataTextBox2.TabIndex = 5;
            this.dataTextBox2.Text = "22";
            this.dataTextBox2.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            this.dataTextBox2.TextChanged += new System.EventHandler(this.dataTextBox_TextChanged);
            this.dataTextBox2.KeyPress += new System.Windows.Forms.KeyPressEventHandler(this.tBox_KeyPress);
            // 
            // dataTextBox3
            // 
            this.dataTextBox3.Location = new System.Drawing.Point(94, 98);
            this.dataTextBox3.MaxLength = 2;
            this.dataTextBox3.Name = "dataTextBox3";
            this.dataTextBox3.Size = new System.Drawing.Size(35, 21);
            this.dataTextBox3.TabIndex = 6;
            this.dataTextBox3.Text = "33";
            this.dataTextBox3.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            this.dataTextBox3.TextChanged += new System.EventHandler(this.dataTextBox_TextChanged);
            this.dataTextBox3.KeyPress += new System.Windows.Forms.KeyPressEventHandler(this.tBox_KeyPress);
            // 
            // dataTextBox4
            // 
            this.dataTextBox4.Location = new System.Drawing.Point(135, 98);
            this.dataTextBox4.MaxLength = 2;
            this.dataTextBox4.Name = "dataTextBox4";
            this.dataTextBox4.Size = new System.Drawing.Size(35, 21);
            this.dataTextBox4.TabIndex = 7;
            this.dataTextBox4.Text = "44";
            this.dataTextBox4.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            this.dataTextBox4.TextChanged += new System.EventHandler(this.dataTextBox_TextChanged);
            this.dataTextBox4.KeyPress += new System.Windows.Forms.KeyPressEventHandler(this.tBox_KeyPress);
            // 
            // dataTextBox5
            // 
            this.dataTextBox5.Location = new System.Drawing.Point(176, 98);
            this.dataTextBox5.MaxLength = 2;
            this.dataTextBox5.Name = "dataTextBox5";
            this.dataTextBox5.Size = new System.Drawing.Size(35, 21);
            this.dataTextBox5.TabIndex = 8;
            this.dataTextBox5.Text = "55";
            this.dataTextBox5.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            this.dataTextBox5.TextChanged += new System.EventHandler(this.dataTextBox_TextChanged);
            this.dataTextBox5.KeyPress += new System.Windows.Forms.KeyPressEventHandler(this.tBox_KeyPress);
            // 
            // dataTextBox6
            // 
            this.dataTextBox6.Location = new System.Drawing.Point(217, 98);
            this.dataTextBox6.MaxLength = 2;
            this.dataTextBox6.Name = "dataTextBox6";
            this.dataTextBox6.Size = new System.Drawing.Size(35, 21);
            this.dataTextBox6.TabIndex = 9;
            this.dataTextBox6.Text = "66";
            this.dataTextBox6.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            this.dataTextBox6.TextChanged += new System.EventHandler(this.dataTextBox_TextChanged);
            this.dataTextBox6.KeyPress += new System.Windows.Forms.KeyPressEventHandler(this.tBox_KeyPress);
            // 
            // dataTextBox7
            // 
            this.dataTextBox7.Location = new System.Drawing.Point(258, 98);
            this.dataTextBox7.MaxLength = 2;
            this.dataTextBox7.Name = "dataTextBox7";
            this.dataTextBox7.Size = new System.Drawing.Size(35, 21);
            this.dataTextBox7.TabIndex = 10;
            this.dataTextBox7.Text = "77";
            this.dataTextBox7.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            this.dataTextBox7.TextChanged += new System.EventHandler(this.dataTextBox_TextChanged);
            this.dataTextBox7.KeyPress += new System.Windows.Forms.KeyPressEventHandler(this.tBox_KeyPress);
            // 
            // dataTextBox8
            // 
            this.dataTextBox8.Location = new System.Drawing.Point(299, 98);
            this.dataTextBox8.MaxLength = 2;
            this.dataTextBox8.Name = "dataTextBox8";
            this.dataTextBox8.Size = new System.Drawing.Size(35, 21);
            this.dataTextBox8.TabIndex = 11;
            this.dataTextBox8.Text = "88";
            this.dataTextBox8.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            this.dataTextBox8.TextChanged += new System.EventHandler(this.dataTextBox_TextChanged);
            this.dataTextBox8.KeyPress += new System.Windows.Forms.KeyPressEventHandler(this.tBox_KeyPress);
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(63, 34);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(44, 12);
            this.label1.TabIndex = 12;
            this.label1.Text = "Format";
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(202, 34);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(48, 12);
            this.label2.TabIndex = 13;
            this.label2.Text = "ID(Hex)";
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(296, 34);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(29, 12);
            this.label3.TabIndex = 14;
            this.label3.Text = "DLC";
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(102, 80);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(145, 12);
            this.label4.TabIndex = 15;
            this.label4.Text = "------DATA (Hex)------";
            // 
            // sendButton
            // 
            this.sendButton.Location = new System.Drawing.Point(356, 100);
            this.sendButton.Name = "sendButton";
            this.sendButton.Size = new System.Drawing.Size(75, 23);
            this.sendButton.TabIndex = 16;
            this.sendButton.Text = "Send";
            this.sendButton.UseVisualStyleBackColor = true;
            this.sendButton.Click += new System.EventHandler(this.sendButton_Click);
            // 
            // monitorRichTextBox
            // 
            this.monitorRichTextBox.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom) 
            | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.monitorRichTextBox.Location = new System.Drawing.Point(12, 135);
            this.monitorRichTextBox.Name = "monitorRichTextBox";
            this.monitorRichTextBox.Size = new System.Drawing.Size(419, 105);
            this.monitorRichTextBox.TabIndex = 17;
            this.monitorRichTextBox.Text = "";
            // 
            // SerialtoCAN_Sample
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(7F, 12F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(443, 252);
            this.Controls.Add(this.monitorRichTextBox);
            this.Controls.Add(this.sendButton);
            this.Controls.Add(this.label4);
            this.Controls.Add(this.label3);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.label1);
            this.Controls.Add(this.dataTextBox8);
            this.Controls.Add(this.dataTextBox7);
            this.Controls.Add(this.dataTextBox6);
            this.Controls.Add(this.dataTextBox5);
            this.Controls.Add(this.dataTextBox4);
            this.Controls.Add(this.dataTextBox3);
            this.Controls.Add(this.dataTextBox2);
            this.Controls.Add(this.dataTextBox1);
            this.Controls.Add(this.idTextBox);
            this.Controls.Add(this.dlcComboBox);
            this.Controls.Add(this.formatComboBox);
            this.Controls.Add(this.menuStrip1);
            this.MainMenuStrip = this.menuStrip1;
            this.MinimumSize = new System.Drawing.Size(459, 251);
            this.Name = "SerialtoCAN_Sample";
            this.Text = "SerialtoCAN_Sample";
            this.Load += new System.EventHandler(this.SerialtoCAN_Sample_Load);
            this.menuStrip1.ResumeLayout(false);
            this.menuStrip1.PerformLayout();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.MenuStrip menuStrip1;
        private System.Windows.Forms.ToolStripMenuItem actionToolStripMenuItem;
        private System.Windows.Forms.ToolStripMenuItem connectToolStripMenuItem;
        private System.Windows.Forms.ToolStripMenuItem disconnectToolStripMenuItem;
        private System.Windows.Forms.ToolStripMenuItem helpToolStripMenuItem;
        private System.Windows.Forms.ToolStripMenuItem aboutToolStripMenuItem;
        private System.Windows.Forms.ComboBox formatComboBox;
        private System.Windows.Forms.ComboBox dlcComboBox;
        private System.Windows.Forms.TextBox idTextBox;
        private System.Windows.Forms.TextBox dataTextBox1;
        private System.Windows.Forms.TextBox dataTextBox2;
        private System.Windows.Forms.TextBox dataTextBox3;
        private System.Windows.Forms.TextBox dataTextBox4;
        private System.Windows.Forms.TextBox dataTextBox5;
        private System.Windows.Forms.TextBox dataTextBox6;
        private System.Windows.Forms.TextBox dataTextBox7;
        private System.Windows.Forms.TextBox dataTextBox8;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.Button sendButton;
        private System.Windows.Forms.RichTextBox monitorRichTextBox;
    }
}

