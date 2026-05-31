// ==========================================
// 1. 物理演算用 2次元ベクトルクラス
// ==========================================
class Vector2D {
    constructor(public x: number = 0, public y: number = 0) {}
    add(v: Vector2D): Vector2D { return new Vector2D(this.x + v.x, this.y + v.y); }
    scale(n: number): Vector2D { return new Vector2D(this.x * n, this.y * n); }
    length(): number { return Math.sqrt(this.x * this.x + this.y * this.y); }
}

// ==========================================
// 2. 航空力学エンジン（機体クラス）
// ==========================================
class Airplane {
    // 状態量
    public position = new Vector2D(100, 200); // 位置 (x, y) ※yは高度
    public velocity = new Vector2D(30, 0);    // 速度ベクトル (vx, vy) ※初期前進速度 30m/s
    public pitch: number = 0;                 // 現在の機首方位 (ラジアン)
    public targetPitch: number = 0;           // マウス入力による目標機首方位
    public throttle: number = 0.7;            // スロットル (0.0 〜 1.0)
    public isStalled: boolean = false;        // 失速フラグ

    // 機体諸元（セスナ172をベースとした定数）
    private readonly mass = 1200;           // 重量 (kg)
    private readonly wingArea = 16.2;       // 翼面積 (m2)
    private readonly maxThrust = 3500;      // 最大推力 (N)
    private readonly rho = 1.225;           // 空気密度 (kg/m3)
    private readonly g = 9.81;              // 重力加速度 (m/s2)

    public update(dt: number) {
        // --- 操縦応答（入力に対する機首回転の慣性） ---
        const rotationSpeed = 2.5; 
        const pitchDiff = this.targetPitch - this.pitch;
        this.pitch += Math.max(-rotationSpeed * dt, Math.min(rotationSpeed * dt, pitchDiff));

        // --- 航空力学計算 ---
        const speed = this.velocity.length();

        // 迎角（AoA: Angle of Attack）の計算 (機首方位と実際の進行方位の差)
        const moveAngle = speed > 0.5 ? Math.atan2(this.velocity.y, this.velocity.x) : this.pitch;
        const aoa = this.pitch - moveAngle;

        // 揚力(cL)・抗力(cD)係数の計算（失速モデルの導入）
        let cL = 0;
        let cD = 0.04; // 基本形状抗力
        const maxAoA = 0.26; // 失速臨界角（約15度）

        if (Math.abs(aoa) < maxAoA) {
            // 通常飛行状態
            cL = 2 * Math.PI * aoa;
            cD += 0.05 * (cL * cL); // 誘導抗力の加算
            this.isStalled = false;
        } else {
            // 失速（ストール）状態：空気の剥離により揚力激減、抗力激増
            cL = (2 * Math.PI * maxAoA) * 0.2 * Math.sign(aoa);
            cD += 0.4; 
            this.isStalled = true;
        }

        // 動圧の計算 (0.5 * rho * v^2)
        const dynamicPressure = 0.5 * this.rho * speed * speed;

        // 各種力の大きさを算出
        const thrustMag = this.throttle * this.maxThrust;
        const liftMag = cL * dynamicPressure * this.wingArea;
        const dragMag = cD * dynamicPressure * this.wingArea;

        // --- 力のベクトル合成 ---
        const forwardX = Math.cos(this.pitch);
        const forwardY = Math.sin(this.pitch);
        const upX = -Math.sin(this.pitch);
        const upY = Math.cos(this.pitch);

        const totalForce = new Vector2D(0, 0);

        // 1. 推力 (機首の向いている方向)
        totalForce.x += forwardX * thrustMag;
        totalForce.y += forwardY * thrustMag;

        // 2. 抗力 (進行方向の逆向き)
        if (speed > 0.1) {
            totalForce.x -= (this.velocity.x / speed) * dragMag;
            totalForce.y -= (this.velocity.y / speed) * dragMag;
        }

        // 3. 揚力 (機首に対して垂直上方向)
        totalForce.x += upX * liftMag;
        totalForce.y += upY * liftMag;

        // 4. 重力 (常に真下)
        totalForce.y -= this.mass * this.g;

        // --- 運動方程式 F = ma => a = F/m ---
        const acceleration = totalForce.scale(1 / this.mass);

        // 速度と位置の更新 (オイラー積分)
        this.velocity = this.velocity.add(acceleration.scale(dt));
        this.position = this.position.add(this.velocity.scale(dt));

        // 地面衝突および着陸判定
        if (this.position.y < 15) {
            this.position.y = 15;
            // 速度が十分低く、ピッチが水平に近ければ着陸、そうでなければクラッシュ（簡易停止）
            if (Math.abs(this.velocity.y) < 5 && Math.abs(this.pitch) < 0.2) {
                this.velocity.y = 0;
                this.pitch = 0;
                // 地面摩擦
                this.velocity.x *= 0.95; 
            } else {
                this.velocity = new Vector2D(0, 0);
            }
        }
    }

    public setThrottle(value: number) {
        this.throttle = Math.max(0, Math.min(1, value));
    }
}

// ==========================================
// 3. グラフィック描画クラス (HTML5 Canvas)
// ==========================================
class SimulatorRenderer {
    private canvas: HTMLCanvasElement;
    private ctx: CanvasRenderingContext2D;

    constructor(canvasId: string) {
        this.canvas = document.getElementById(canvasId) as HTMLCanvasElement;
        this.ctx = this.canvas.getContext('2d')!;
    }

    public render(airplane: Airplane, mouseX: number, mouseY: number) {
        // 画面クリア
        this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);

        // 背景・地面の描画
        this.drawEnvironment();

        // 航空機の描画 (座標系をCanvas標準から物理座標［Y軸反転］に変換)
        this.ctx.save();
        // 飛行機が常に画面の横幅30%の位置に固定されるようカメラを追従（背景ループ対応）
        const cameraX = airplane.position.x - 240; 
        this.ctx.translate(airplane.position.x - cameraX, this.canvas.height - airplane.position.y);

        this.drawAirplane(airplane);
        this.ctx.restore();

        // HUD（計器情報）と操縦インジケーターの描画
        this.drawHUD(airplane, mouseX, mouseY);
    }

    private drawEnvironment() {
        // 地面 (グリッドや緑の線で動きをわかりやすく)
        this.ctx.fillStyle = '#4CAF50';
        this.ctx.fillRect(0, this.canvas.height - 15, this.canvas.width, 15);
    }

    private drawAirplane(airplane: Airplane) {
        this.ctx.save();
        this.ctx.rotate(-airplane.pitch); // Canvasの回転方向へ合わせる

        // 機体カラー（失速時は赤くなる）
        this.ctx.fillStyle = airplane.isStalled ? '#ff1744' : '#37474f';

        // 簡易的な機体の形状（飛行機型）
        this.ctx.beginPath();
        this.ctx.moveTo(35, 0);     // 機首
        this.ctx.lineTo(-15, -8);   // アッパーボディ
        this.ctx.lineTo(-30, -20);  // 尾翼トップ
        this.ctx.lineTo(-25, 0);    // 尾翼ボトム
        this.ctx.lineTo(-15, 8);    // ロワーボディ
        this.ctx.closePath();
        this.ctx.fill();

        // 主翼
        this.ctx.lineWidth = 4;
        this.ctx.strokeStyle = '#78909c';
        this.ctx.beginPath();
        this.ctx.moveTo(-5, 0);
        this.ctx.lineTo(-10, -30);
        this.ctx.moveTo(-5, 0);
        this.ctx.lineTo(-10, 30);
        this.ctx.stroke();

        this.ctx.restore();
    }

    private drawHUD(airplane: Airplane, mouseX: number, mouseY: number) {
        // 文字情報
        this.ctx.fillStyle = '#263238';
        this.ctx.font = 'bold 14px monospace';
        this.ctx.fillText(`高度 (ALT): ${Math.round(airplane.position.y)} m`, 20, 35);
        this.ctx.fillText(`速度 (SPD): ${Math.round(airplane.velocity.length() * 3.6)} km/h`, 20, 60);
        this.ctx.fillText(`出力 (THR): ${Math.round(airplane.throttle * 100)} %`, 20, 85);

        // 失速警告アラート
        if (airplane.isStalled && airplane.position.y > 16) {
            this.ctx.fillStyle = '#d50000';
            this.ctx.font = 'blink bold 20px monospace';
            this.ctx.fillText('⚠️ STALL (失速)', 20, 125);
        }

        // 右側のジョイスティック（マウス位置）インジケーター
        const uiX = this.canvas.width - 60;
        const uiY = this.canvas.height / 2;
        
        this.ctx.save();
        this.ctx.strokeStyle = 'rgba(0, 0, 0, 0.2)';
        this.ctx.lineWidth = 2;
        this.ctx.beginPath();
        this.ctx.arc(uiX, uiY, 40, 0, Math.PI * 2);
        this.ctx.stroke();

        // マウスの現在入力量をプロット
        const inputY = (mouseY - uiY) / (this.canvas.height / 2);
        const clampedY = Math.max(-1, Math.min(1, inputY));
        
        this.ctx.fillStyle = '#1e88e5';
        this.ctx.beginPath();
        this.ctx.arc(uiX, uiY + (clampedY * 40), 6, 0, Math.PI * 2);
        this.ctx.fill();
        this.ctx.restore();
    }
}

// ==========================================
// 4. メインループ・アプリケーション駆動（修正版）
// ==========================================
const airplane = new Airplane();
const renderer = new SimulatorRenderer('simCanvas');
const canvasElement = document.getElementById('simCanvas') as HTMLCanvasElement;

let currentMouseX = 0;
let currentMouseY = canvasElement.height / 2;

// --- キーの入力状態を記録するオブジェクト ---
const keyStates = {
    ArrowLeft: false,
    ArrowRight: false
};

// --- 【マウス移動】ピッチ操作 ---
canvasElement.addEventListener('mousemove', (e) => {
    const rect = canvasElement.getBoundingClientRect();
    currentMouseX = e.clientX - rect.left;
    currentMouseY = e.clientY - rect.top;

    const centerY = canvasElement.height / 2;
    const inputY = (currentMouseY - centerY) / (canvasElement.height / 2);

    const maxPitch = 35 * (Math.PI / 180);
    airplane.targetPitch = -inputY * maxPitch; 
});

// --- 【キーボード】押されたフラグをONにする ---
window.addEventListener('keydown', (e) => {
    if (e.key === 'ArrowLeft' || e.key === 'ArrowRight') {
        keyStates[e.key] = true;
    }
});

// --- 【キーボード】離されたらフラグをOFFにする ---
window.addEventListener('keyup', (e) => {
    if (e.key === 'ArrowLeft' || e.key === 'ArrowRight') {
        keyStates[e.key] = false;
    }
});


let lastTime = performance.now();

function step(currentTime: number) {
    const dt = (currentTime - lastTime) / 1000;
    lastTime = currentTime;

    const cappedDt = Math.min(dt, 0.1);

    // --- 【ここがポイント】押し続け状態に応じて、毎フレーム少しずつスロットルを増減 ---
    // 1秒間押し続けると、最大（1.0）の40%分（0.4）スロットルが変化する計算
    const throttleSpeed = 0.4; 

    if (keyStates.ArrowRight) {
        // 右矢印を押し続けている間、滑らかに加速
        airplane.setThrottle(airplane.throttle + throttleSpeed * cappedDt);
    }
    if (keyStates.ArrowLeft) {
        // 左矢印を押し続けている間、滑らかに減速
        airplane.setThrottle(airplane.throttle - throttleSpeed * cappedDt);
    }

    // 物理演算と描画の実行
    airplane.update(cappedDt);
    renderer.render(airplane, currentMouseX, currentMouseY);

    requestAnimationFrame(step);
}

// ループ始動
requestAnimationFrame(step);