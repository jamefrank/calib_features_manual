#ifndef CUSTOM_INTERACTOR_STYLE_H
#define CUSTOM_INTERACTOR_STYLE_H

// 必须包含这些 VTK 头文件
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkObjectFactory.h>  // ⬅️ 必须加！提供 vtkStandardNewMacro 定义
#include <vtkRenderWindowInteractor.h>

class MyVtkInteractorStyle : public vtkInteractorStyleTrackballCamera
{
public:
    static MyVtkInteractorStyle* New();
    vtkTypeMacro(MyVtkInteractorStyle, vtkInteractorStyleTrackballCamera);

protected:
    MyVtkInteractorStyle() = default;
    ~MyVtkInteractorStyle() override = default;

    void OnRightButtonDown() override;
    void OnRightButtonUp() override;

    void OnChar() override;
};


#endif // CUSTOM_INTERACTOR_STYLE_H