#include "TxGeneratorWindow.h"
#include "core/MeasurementNetwork.h"
#include "ui_TxGeneratorWindow.h"

#include <QDomDocument>
#include <QTimer>
#include <core/Backend.h>
#include <driver/CanInterface.h>


TxGeneratorWindow::TxGeneratorWindow(QWidget *parent, Backend &backend) :
    ConfigurableWidget(parent),
    ui(new Ui::TxGeneratorWindow),
    _backend(backend)
{
    ui->setupUi(this);

    ui->treeWidget->setHeaderLabels(QStringList() << tr("Nr") << tr("Name") << tr("Cycle Time"));
    ui->treeWidget->setColumnWidth(0, 40);
    ui->treeWidget->setColumnWidth(1, 160);

    // Timer for repeating messages
    _SendTimer = new QTimer(this);
    _SendTimer->setTimerType(Qt::PreciseTimer);
    //_SendTimer->setInterval(1);
    connect(_SendTimer, SIGNAL(timeout()), this, SLOT(SendTimer_timeout()));

    QTreeWidgetItem *item = new QTreeWidgetItem(ui->treeWidget);
    item->setText(column_nr, QString().number(1));
    item->setText(column_name, "Name_test");
    item->setText(column_cycletime, QString().number(100));
}

TxGeneratorWindow::~TxGeneratorWindow()
{
    delete ui;
}

bool TxGeneratorWindow::saveXML(Backend &backend, QDomDocument &xml, QDomElement &root)
{
    if (!ConfigurableWidget::saveXML(backend, xml, root)) { return false; }
    root.setAttribute("type", "TxGeneratorWindow");
    return true;
}

bool TxGeneratorWindow::loadXML(Backend &backend, QDomElement &el)
{
    if (!ConfigurableWidget::loadXML(backend, el)) { return false; }
    return true;
}

void TxGeneratorWindow::SendTimer_timeout()
{

}

void TxGeneratorWindow::update()
{
    //fprintf(stderr, "update\r\n");
}

void TxGeneratorWindow::on_treeWidget_itemClicked(QTreeWidgetItem *item, int column)
{

}

void TxGeneratorWindow::on_btnAdd_released()
{
    MeasurementSetup &setup = _backend.getSetup();

    foreach (MeasurementNetwork *network, setup.getNetworks())
    {
        foreach (pCanDb db, network->_canDbs)
        {
            auto name = db->getFileName();
            auto msgs = db->getMessageList();
        }
    }
}

