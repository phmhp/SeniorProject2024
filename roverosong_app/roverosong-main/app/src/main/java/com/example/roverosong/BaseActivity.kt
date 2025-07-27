package com.example.roverosong

import android.app.AlertDialog
import android.os.Bundle
import android.util.Log
import android.view.Gravity
import android.widget.*
import androidx.appcompat.app.AppCompatActivity
import com.google.firebase.database.*

open class BaseActivity : AppCompatActivity() {

    private lateinit var database: DatabaseReference
    private lateinit var batteryStatusText: TextView  // 배터리 상태 표시 TextView
    private lateinit var robotStatusText: TextView  // 로봇 상태 표시 TextView
    private lateinit var batteryLayout: LinearLayout

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        // Firebase Database 참조 (경로: robot_battery/battery)
        database = FirebaseDatabase.getInstance().getReference("robot_battery/battery")

        // Firebase에서 배터리 상태를 실시간으로 가져오기
        database.addValueEventListener(object : ValueEventListener {
            override fun onDataChange(snapshot: DataSnapshot) {
                val batteryLevel = snapshot.getValue(Int::class.java) ?: 0  // null 방지 기본값 0
                Log.d("Firebase", "배터리 상태 업데이트: $batteryLevel%")  // 로그 추가 (디버깅용)

                // 배터리 상태 업데이트
                batteryStatusText.text = "$batteryLevel%"

                // 배터리 잔량에 따라 로봇 상태 변경
                when {
                    batteryLevel > 30 -> robotStatusText.text = "🤖🔋"  // 정상 상태
                    batteryLevel <= 30 -> {
                        robotStatusText.text = "🤖🪫"  // 배터리 부족
                        showLowBatteryPopup()
                    }
                }
            }

            override fun onCancelled(error: DatabaseError) {
                Log.e("FirebaseError", "배터리 상태를 가져오는 중 오류 발생: ${error.message}")
            }
        })
    }

    override fun setContentView(layoutResID: Int) {
        super.setContentView(layoutResID) // 기존 레이아웃 설정
        addBatteryStatusLayout() // 배터리 + 로봇 이모지 UI 추가
    }

    // 배터리 상태 + 로봇 이모지 UI
    private fun addBatteryStatusLayout() {
        batteryLayout = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL // 가로 정렬
        }

        // 로봇 이모지 (작게 조정)
        robotStatusText = TextView(this).apply {
            text = "🤖" // 기본 이모지 (정상 상태)
            textSize = 22f  //크기 줄임
            setTextColor(resources.getColor(android.R.color.black, theme))
        }

        // 배터리 상태 표시
        batteryStatusText = TextView(this).apply {
            text = "--%"
            textSize = 18f  //배터리 텍스트 크기 유지
            setTextColor(resources.getColor(android.R.color.black, theme))
        }

        // 레이아웃에 추가 (로봇 → 배터리 순서)
        batteryLayout.addView(robotStatusText)
        batteryLayout.addView(batteryStatusText)

        // 위치 설정
        val layoutParams = RelativeLayout.LayoutParams(
            RelativeLayout.LayoutParams.WRAP_CONTENT,
            RelativeLayout.LayoutParams.WRAP_CONTENT
        ).apply {
            addRule(RelativeLayout.ALIGN_PARENT_START) // 왼쪽 정렬
            addRule(RelativeLayout.ALIGN_PARENT_TOP) // 상단 정렬
            setMargins(20, 50, 0, 0) //왼쪽 여백 추가
        }

        val rootView = findViewById<FrameLayout>(android.R.id.content)
        rootView.addView(batteryLayout, layoutParams) //최상위 레이아웃에 추가
    }

    // 배터리 부족 팝업 표시
    private fun showLowBatteryPopup() {
        val builder = AlertDialog.Builder(this)
        builder.setTitle("⚠️ 배터리 부족 경고")
        builder.setMessage("로봇의 배터리가 30% 이하입니다. 충전이 필요합니다.")
        builder.setPositiveButton("확인") { dialog, _ -> dialog.dismiss() }
        val dialog = builder.create()
        dialog.show()
    }}