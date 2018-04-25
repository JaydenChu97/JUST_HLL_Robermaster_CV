#ifdef DEBUG
    /*Qt库*/
    #include "mainwindow.h"
    #include <QApplication>
#else
    /*主控制库*/
    #include "main_control.h"
#endif

int main(int argc, char *argv[])
{

#ifdef DEBUG
    QApplication a(argc, argv);
    MainWindow instance;
    instance.show();
#else
    HCVC::MainControl instance;
#endif

    try
    {
        //F:\\Robomaster\\视觉素材\\视觉素材\\炮台素材红车旋转-ev-0.MOV
        instance.run("camera");
    }
    catch(const string& hint)
    {
        cout << hint << endl;
    }
    catch(const Exception& exception)
    {
        cout << exception.what() << endl;
    }

#ifdef DEBUG
    instance.close();

    return a.exec();
#else
    return 0;
#endif
}
