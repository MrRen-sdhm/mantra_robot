#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

namespace Ui {
class Form;
}

class Form : public QMainWindow
{
    Q_OBJECT

public:
    explicit Form(QWidget *parent = nullptr);
    ~Form();

private:
    Ui::Form *ui;

private slots:
    void power();
    void save_state();
    void slide_moved();
    void reset_arm();
    void selectionchange();
    void set_home();
    void back_home();

    void joint1_minus();
    void joint1_minus_done();
    void joint1_plus();
    void joint1_plus_done();
    void joint2_minus();
    void joint2_minus_done();
    void joint2_plus();
    void joint2_plus_done();
    void joint3_minus();
    void joint3_minus_done();
    void joint3_plus();
    void joint3_plus_done();
    void joint4_minus();
    void joint4_minus_done();
    void joint4_plus();
    void joint4_plus_done();
    void joint5_minus();
    void joint5_minus_done();
    void joint5_plus();
    void joint5_plus_done();
    void joint6_minus();
    void joint6_minus_done();
    void joint6_plus();
    void joint6_plus_done();
    void joint7_minus();
    void joint7_minus_done();
    void joint7_plus();
    void joint7_plus_done();
};

#endif // MAINWINDOW_H
