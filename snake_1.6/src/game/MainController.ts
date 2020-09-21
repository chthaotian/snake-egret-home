import { Behaviour, Application, EngineFactory, GameEntity } from "@egret/engine";
import { property, EditType, serializedField, getItemsFromEnum } from "@egret/core";
import { component } from "@egret/ecs";
import { MeshFilter, DefaultMeshes, MeshRenderer, DefaultMaterials, Material } from "C:/Users/Administrator/AppData/Roaming/egretpro/engine/release/1.6.0/packages/render";
import { InputManager, InputCode, Input, InputState } from "C:/Users/Administrator/AppData/Roaming/egretpro/engine/release/1.6.0/packages/input";
import * as engine from "@egret/engine";
import * as input from "@egret/input";

@component()
export class MainController extends Behaviour {
    //所有实体-蛇身，蛇头
    private _entitys: Array<GameEntity> = [];
    //存储对应的颜色
    // private _entitysArray: Array<uint> = [];
    //蛇头
    private _head: GameEntity = null;
    //蛇身的长度（消掉的也算，或者说是保留初始长度在内的蛇身球体的总个数）
    private _totolCount: uint = 0;
    //存的是速度变化
    private _speedArray: Array<uint> = [3,4,5,6,10,12,15,20,30,60];
    //蛇头行走轨迹点
    private _headTrack = [];
    //储存随机出现的点
    private _randomPoint = [];
    //储存按住的时候的点位置
    private _xyPoint = [];
    //颜色存储器 红橙黄绿青蓝紫
    private colors = [
        {r:1,g:1,b:1},
        {r:1,g:0,b:0},
        {r:1,g:0.5,b:0},
        {r:1,g:1,b:0},
        {r:0,g:1,b:0},
        {r:0,g:1,b:1},
        {r:0,g:0,b:1},
        {r:1,g:0,b:1}
    ];

    //是否继续游戏开关
    private key = true;
    //滑动的角度
    private _holdsin = 1;
    private _holdcos = 0;
    //常量
    private _z = 0;
     //头部index
    private _headIndex = 0;
    //1秒调用的此时
    private _fixedTime =60;
    //头部一个初始值，为了防止和第一个点拐弯时碰撞
    private _initIndex = 0;
    //消除的次数
    private _delNum = 0;
    //宽
    private _width = 38;
    //高
    private _height = 34;
    //球的直径
    private _SPHERE = 1;
    //蛇头轨迹
    //蛇头延一个方向行走的距离，当按wasd时清零并记录轨迹
    //整个游戏的过程中都在遍历轨迹
    //每个小球设置是否最后一个，和各自的轨迹，以此来和球体的轨迹对比，决定走向
    // @property(EditType.Enum, { listItems: getItemsFromEnum(((input as any).InputCode)) }) // TODO
    // @serializedField
    // public inputType: InputCode = InputCode.PenEraser;
    // private _input: Input | null = null;
    private _input = null;
    public speed: float = this._speedArray[0];

    onAwake(){
        Application.instance.stage.size = {w:1080,h:1920};
    }
    //执行一次
    public onStart(): void {
        //MESH_NORMAL MESH_LAMBERT MESH_PHONG可做头
        if(this._head == null){
            // this._head = this.createEntity(0,0,this._z,DefaultMaterials.MESH_NORMAL,1,0,0,1,DefaultMeshes.CUBE);
            this._head = this.createEntity(0,0+this._initIndex,this._z,DefaultMaterials.MESH_LAMBERT,this.colors[0].r,this.colors[0].g,this.colors[0].b,1,DefaultMeshes.SPHERE);
            this._entitys.push(this._head);
            this._head["colorIndex"] = 0;
            this._totolCount++;
 
            const firstBody = this.createEntity(0,-1,this._z,DefaultMaterials.MESH_LAMBERT,this.colors[1].r,this.colors[1].g,this.colors[1].b,1,DefaultMeshes.SPHERE);
            this._entitys.push(firstBody);
            firstBody["colorIndex"] = 1;
            this._totolCount++;
      
            const secondBody = this.createEntity(0,-2,this._z,DefaultMaterials.MESH_LAMBERT,this.colors[2].r,this.colors[2].g,this.colors[2].b,1,DefaultMeshes.SPHERE);
            this._entitys.push(secondBody);
            secondBody["colorIndex"] = 2;
            this._totolCount++;
            console.log(secondBody);
      
            const threeBody = this.createEntity(0,-3,this._z,DefaultMaterials.MESH_LAMBERT,this.colors[3].r,this.colors[3].g,this.colors[3].b,1,DefaultMeshes.SPHERE);
            this._entitys.push(threeBody);
            threeBody["colorIndex"] = 3;
            this._totolCount++;
     
            const thirdBody = this.createEntity(0,-4,this._z,DefaultMaterials.MESH_LAMBERT,this.colors[4].r,this.colors[4].g,this.colors[4].b,1,DefaultMeshes.SPHERE);
            this._entitys.push(thirdBody);
            thirdBody["colorIndex"] = 4;
            this._totolCount++;
           
            let smallLength = FloatDiv(this.speed,this._fixedTime);
            for(let i = -4;i<=this._initIndex;i=FloatAdd(i,smallLength)){
                let map = {"x":0,"y":i};
                this._headTrack.unshift(map);
            }
            this.random3Entity();
            // const inputManager = Application.instance.globalEntity.getComponent(InputManager)!;
            this._input = Application.instance.globalEntity.getComponent(InputManager);
        }
        // this.createEntity(-4,2,0,DefaultMaterials.MESH_LAMBERT,1,0,0,1,DefaultMeshes.CUBE);
        // this.createEntity(-6,-4,0,DefaultMaterials.MESH_PHONG,1,0,0,1,DefaultMeshes.CUBE);
        //MESH_LAMBERT MESH_PHYSICAL MESH_STANDARD 做身体
        // this.createEntity(-4,2,0,DefaultMaterials.MESH_LAMBERT,1,0,0,1,DefaultMeshes.SPHERE);
        // this.createEntity(-5,-3,0,DefaultMaterials.MESH_PHYSICAL,1,0,0,1,DefaultMeshes.SPHERE);
        // this.createEntity(-4,-2,0,DefaultMaterials.MESH_STANDARD,1,0,0,1,DefaultMeshes.SPHERE);
    }

    public random3Entity(){
        let x1 = Math.random() * this._width - this._width/2;
        let y1 = Math.random() * this._height -this._height/2;
        let colorIndex = this.randomNum(1,7);
        let random1 = this.createEntity(x1,y1,this._z,DefaultMaterials.MESH_LAMBERT,this.colors[colorIndex].r,this.colors[colorIndex].g,this.colors[colorIndex].b,1,DefaultMeshes.SPHERE);
        random1["colorIndex"] = colorIndex;
        this._randomPoint.push(random1);
        let x2 = Math.random() * this._width - this._width/2;
        let y2 = Math.random() * this._height -this._height/2;
        while(Math.sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1))<=1){
            x2 = Math.random() * this._width - this._width/2;
            y2 = Math.random() * this._height -this._height/2;
        }
        let colorIndex2 = this.randomNum(1,7);
        let random2 = this.createEntity(x2,y2,this._z,DefaultMaterials.MESH_LAMBERT,this.colors[colorIndex2].r,this.colors[colorIndex2].g,this.colors[colorIndex2].b,1,DefaultMeshes.SPHERE);
        random2["colorIndex"] = colorIndex2;
        this._randomPoint.push(random2);
        let x3 = Math.random() * this._width - this._width/2;
        let y3 = Math.random() * this._height -this._height/2;
        while(Math.sqrt((x3 - x1) * (x3 - x1) + (y3 - y1) * (y3 - y1))<=1||Math.sqrt((x3 - x2) * (x3 - x2) + (y3 - y2) * (y3 - y2))<=1){
            x3 = Math.random() * this._width - this._width/2;
            y3 = Math.random() * this._height -this._height/2;
        }
        let colorIndex3 = this.randomNum(1,7);
        let random3 = this.createEntity(x3,y3,this._z,DefaultMaterials.MESH_LAMBERT,this.colors[colorIndex3].r,this.colors[colorIndex3].g,this.colors[colorIndex3].b,1,DefaultMeshes.SPHERE);
        random3["colorIndex"] = colorIndex3;
        this._randomPoint.push(random3);
    };
//随机数
    public randomNum(minNum,maxNum){ 
        switch(arguments.length){ 
            case 1: 
                return parseInt(Math.random()*minNum+1+"",10); 
            break; 
            case 2: 
                return parseInt(Math.random()*(maxNum-minNum+1)+minNum,10); 
            break; 
                default: 
                    return 0; 
                break; 
        } 
    }

    public createEntity(x=0,y=0,z=0,m=DefaultMaterials.MESH_BASIC,r=1,g=0,b=0,a=1,t = DefaultMeshes.CUBE){
        const cube  = EngineFactory.createGameEntity3D(`Pointer`, {
            
        });
        cube.transform.setLocalPosition(
            x,
            y,
            z
        );
        const material = Material.create(m);
        material.setColor({
            r:r,
            g:g,
            b:b,
            a:a
        })
        const meshFilter = cube.addComponent(MeshFilter);
        meshFilter.mesh = t;
        const meshRenderer = cube.addComponent(MeshRenderer);
        meshRenderer.material = material;
        meshRenderer.castShadows = true;
        meshRenderer.receiveShadows = true;
        cube["_totolCount"] = this._totolCount;
        return cube;

    };
    //0.016 1秒的60分之一
    onFixedUpdate(dt){
        // console.log(this.randomNum(0,6));
        if (this._input === null) {
            return;
        }
        let points = this._input.getPointers(InputState.Hold);
        if(points.length != 0){
            let xy = {
                x:points[0].position.x,
                y:points[0].position.y
            }
            if(this._xyPoint.length>=5){
                let point = this._xyPoint[this._xyPoint.length-1];
                this._xyPoint = [];
                this._xyPoint.push(point);
                this._xyPoint.push(xy);
                let _x = points[0].position.x - this._xyPoint[0].x;
                let _y = points[0].position.y - this._xyPoint[0].y;
                let _z = Math.sqrt((_x) * (_x) + (_y) * (_y));
                if(_z!=0){
                    this._holdsin = _y/_z * (-1);
                    this._holdcos = _x/_z;
                }
            }else{
                this._xyPoint.push(xy);
            }
            
        }else{
            this._xyPoint = [];
        }
        if(this._totolCount>25){
            this.speed = this._speedArray[8];
        }else if(this._totolCount>20){
            this.speed = this._speedArray[7];
        }else if(this._totolCount>15){
            this.speed = this._speedArray[6];
        }else if(this._totolCount>10){
            this.speed = this._speedArray[5];
        }else if(this._totolCount>8){
            this.speed = this._speedArray[4];
        }

        if(this._head!=null){
            let x = this._head.transform.localPosition.x;
            let y = this._head.transform.localPosition.y;
    
            x += this.speed * dt * this._holdcos;
            y += this.speed * dt * this._holdsin;
         
            let p1 = this._entitys[this._headIndex].transform.localPosition;
            // for (let i in this._entitys) {
            //     if(Number(i)!=0){
            //         let p2 = this._entitys[Number(i)].transform.localPosition;
            //         let distance = Math.sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
            //         if(distance<1){
            //             key = false;
            //         }
            //     }
            // }
            let isAdd = false;
            let randomPointIndex = 0;
            for (let i in this._randomPoint) {
                let p2 = this._randomPoint[Number(i)].transform.localPosition;
                let distance = Math.sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
                if(distance<1){
                    isAdd = true;
                    randomPointIndex = Number(i);
                    // this._entitys[this._totolCount] = this.createEntity(0,0,this._z,DefaultMaterials.MESH_LAMBERT,0.7,0.3,0.3,1,DefaultMeshes.SPHERE);
                    this._entitys.push(this._randomPoint[Number(i)]);
                    this._totolCount++;
                }
            }
            if(isAdd){
                for (let i in this._randomPoint) {
                    if(Number(i) != Number(randomPointIndex)){
                        this._randomPoint[Number(i)].destroy();
                    }
                }
                this._randomPoint = [];
                this.random3Entity();
                //判断后边3个颜色是否相同，相同则删除
                if(this._entitys[this._entitys.length-1]["colorIndex"] == this._entitys[this._entitys.length-2]["colorIndex"]
                && this._entitys[this._entitys.length-1]["colorIndex"] == this._entitys[this._entitys.length-3]["colorIndex"]){
                    this._entitys[this._entitys.length-1].destroy();
                    this._entitys[this._entitys.length-2].destroy();
                    this._entitys[this._entitys.length-3].destroy();
                    this._entitys.splice(this._entitys.length-3,3);
                }
            }
            if(x>(this._width/2-0.5)||x<((-1)*this._width/2+0.5)||y>(this._height/2-0.5)||y<(-1)*this._height/2+0.5){
                this.key = false;
                //游戏结束画面
            }
            //更新位置，并记录坐标
            if(this.key){
                let index = this._fixedTime/this.speed;
                for (let i in this._entitys) {
                    if(Number(i)!=0){
                        this._entitys[Number(i)].transform.setLocalPosition(
                            this._headTrack[index].x,
                            this._headTrack[index].y,
                            this._z
                        );
                        index+=this._fixedTime/this.speed;
                    }
                } 
                this._head.transform.setLocalPosition(
                    x,
                    y,
                    this._z
                );
                let map = {"x":x,"y":y};
                this._headTrack.unshift(map);
                //清除多余的轨迹点，防止内存溢出
                let delIndex = index + 5*(this._fixedTime/this.speed);
                if(this._headTrack.length>delIndex){
                    this._headTrack.splice(delIndex,this._headTrack.length-delIndex);
                }
            }else{
                console.log("game over!");
            }
        }
    }
}
//浮点数加法运算 -2.55 0.05
function FloatAdd(arg1,arg2){
    let r1,r2,m;
    try{r1=arg1.toString().split(".")[1].length}catch(e){r1=0}
    try{r2=arg2.toString().split(".")[1].length}catch(e){r2=0}
    m=Math.pow(10,Math.max(r1,r2));
    return (FloatMul(arg1,m)+FloatMul(arg2,m))/m;
}

//浮点数减法运算
function FloatSub(arg1,arg2){
    let r1,r2,m;
    try{r1=arg1.toString().split(".")[1].length}catch(e){r1=0}
    try{r2=arg2.toString().split(".")[1].length}catch(e){r2=0}
    m=Math.pow(10,Math.max(r1,r2));
    return ((FloatMul(arg1,m)-FloatMul(arg2,m))/m);
}

//浮点数乘法运算
function FloatMul(arg1,arg2)
{
    let m=0,s1=arg1.toString(),s2=arg2.toString();
    try{m+=s1.split(".")[1].length}catch(e){}
    try{m+=s2.split(".")[1].length}catch(e){}
    return Number(s1.replace(".",""))*Number(s2.replace(".",""))/Math.pow(10,m);
}


//浮点数除法运算
function FloatDiv(arg1,arg2){
    var t1=0,t2=0,r1,r2;
    try{t1=arg1.toString().split(".")[1].length}catch(e){}
    try{t2=arg2.toString().split(".")[1].length}catch(e){}
    
    r1=Number(arg1.toString().replace(".",""));
    r2=Number(arg2.toString().replace(".",""));
    return (r1/r2)*Math.pow(10,t1-t2);
    
}