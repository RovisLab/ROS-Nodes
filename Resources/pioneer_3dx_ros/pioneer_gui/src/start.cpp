#include "start.h"
#include "ui_start.h"

Start::Start(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Start)
{
    ui->setupUi(this);

    connect(ui->real, SIGNAL(clicked()), this->parent(), SLOT(show_real_tool()));
    connect(ui->sim, SIGNAL(clicked()), this->parent(), SLOT(show_simulation_tool()));
}

Start::~Start()
{
    delete ui;
}
