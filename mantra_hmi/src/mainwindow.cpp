#include "include/mainwindow.h"
#include "include/mainwindow_ui.h"

Form::Form(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::Form)
{
    ui->setupUi(this);
}

void Form::power() {

}

void Form::save_state() {

}

void Form::slide_moved() {

}

void Form::reset_arm() {

}

void Form::selectionchange() {

}

void Form::set_home() {

}

void Form::back_home() {

}

void Form::joint1_minus() {

}

void Form::joint1_minus_done() {

}

void Form::joint1_plus() {

}

void Form::joint1_plus_done() {

}

void Form::joint2_minus() {

}

void Form::joint2_minus_done() {

}

void Form::joint2_plus() {

}

void Form::joint2_plus_done() {

}
void Form::joint3_minus() {

}

void Form::joint3_minus_done() {

}

void Form::joint3_plus() {

}

void Form::joint3_plus_done() {

}
void Form::joint4_minus() {

}

void Form::joint4_minus_done() {

}

void Form::joint4_plus() {

}

void Form::joint4_plus_done() {

}
void Form::joint5_minus() {

}

void Form::joint5_minus_done() {

}

void Form::joint5_plus() {

}

void Form::joint5_plus_done() {

}
void Form::joint6_minus() {

}

void Form::joint6_minus_done() {

}

void Form::joint6_plus() {

}

void Form::joint6_plus_done() {

}

void Form::joint7_minus() {

}

void Form::joint7_minus_done() {

}

void Form::joint7_plus() {

}

void Form::joint7_plus_done() {

}

Form::~Form()
{
    delete ui;
}
