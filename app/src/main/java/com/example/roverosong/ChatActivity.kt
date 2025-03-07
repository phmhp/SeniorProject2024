package com.example.roverosong

import android.app.Activity
import android.app.Dialog
import android.content.Intent
import android.os.Bundle
import android.speech.RecognizerIntent
import android.speech.tts.TextToSpeech
import android.view.Gravity
import android.view.LayoutInflater
import android.widget.*
import androidx.appcompat.app.AppCompatActivity
import com.google.firebase.database.*
import java.util.*

class ChatActivity : BaseActivity(), TextToSpeech.OnInitListener {

    private lateinit var database: DatabaseReference
    private lateinit var micButton: ImageButton
    private lateinit var responseText: TextView
    private lateinit var userSpeechText: TextView
    private lateinit var tts: TextToSpeech  // TTS 추가
    private lateinit var helpButton: Button  // ❓ 버튼 추가

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_chat)

        database = FirebaseDatabase.getInstance().getReference("UserResults/chat")
        micButton = findViewById(R.id.micButton)
        responseText = findViewById(R.id.responseText)
        userSpeechText = findViewById(R.id.userSpeechText)
        helpButton = findViewById(R.id.helpButton)  // ❓ 버튼 초기화

        // TTS 엔진 초기화
        tts = TextToSpeech(this, this)

        //액티비티 시작 시 팝업 자동 표시
        showQuestionsPopup()

        // 음성 인식 버튼 클릭 시 실행
        micButton.setOnClickListener {
            startVoiceRecognition()
        }

        // ? 버튼 클릭 시 팝업 열기
        helpButton.setOnClickListener {
            showQuestionsPopup()
        }

        // Firebase에서 로봇의 응답을 실시간으로 감지하여 표시 및 음성 출력
        database.child("answer").addValueEventListener(object : ValueEventListener {
            override fun onDataChange(snapshot: DataSnapshot) {
                val answer = snapshot.getValue(String::class.java)
                responseText.text = answer ?: "로봇이 아직 대답을 준비 중이에요."

                // TTS로 로봇이 음성으로 말하기
                answer?.let { speakAnswer(it) }
            }

            override fun onCancelled(error: DatabaseError) {
                responseText.text = "오류 발생: ${error.message}"
            }
        })
    }

    // '이렇게 질문해보세요!' 팝업 띄우기
    private fun showQuestionsPopup() {
        val dialog = Dialog(this)
        val view = LayoutInflater.from(this).inflate(R.layout.questions_popup, null)
        dialog.setContentView(view)

        val closeButton = view.findViewById<Button>(R.id.closePopupButton)
        closeButton.setOnClickListener {
            dialog.dismiss()
        }

    // 팝업 창 크기 및 위치 조정
        dialog.window?.setLayout(
            (resources.displayMetrics.widthPixels * 0.85).toInt(),
            (resources.displayMetrics.heightPixels * 0.4).toInt()
        )
        dialog.window?.setGravity(Gravity.CENTER)  // 화면 중앙 배치
        dialog.window?.setBackgroundDrawableResource(android.R.color.transparent) // 배경 투명하게 설정
        dialog.show()
    }

    // 음성 인식 실행
    private fun startVoiceRecognition() {
        val intent = Intent(RecognizerIntent.ACTION_RECOGNIZE_SPEECH)
        intent.putExtra(RecognizerIntent.EXTRA_LANGUAGE_MODEL, RecognizerIntent.LANGUAGE_MODEL_FREE_FORM)
        intent.putExtra(RecognizerIntent.EXTRA_LANGUAGE, "ko-KR")
        intent.putExtra(RecognizerIntent.EXTRA_PROMPT, "말씀해주세요...")

        try {
            startActivityForResult(intent, 100)  // 음성 인식 실행
        } catch (e: Exception) {
            Toast.makeText(this, "음성 인식을 사용할 수 없습니다.", Toast.LENGTH_SHORT).show()
        }
    }

    // 음성 인식 결과 받기 & 사용자 질문 표시
    override fun onActivityResult(requestCode: Int, resultCode: Int, data: Intent?) {
        super.onActivityResult(requestCode, resultCode, data)
        if (requestCode == 100 && resultCode == Activity.RESULT_OK) {
            val result = data?.getStringArrayListExtra(RecognizerIntent.EXTRA_RESULTS)
            val recognizedText = result?.get(0) ?: ""

            // 사용자가 말한 내용 화면에 표시
            userSpeechText.text = "🙋‍♂️ 사용자: $recognizedText"

            // Firebase로 전송
            sendQuestionToFirebase(recognizedText)
        }
    }

    // Firebase에 질문 저장
    private fun sendQuestionToFirebase(question: String) {
        database.child("question").setValue(question)
        responseText.text = "로봇이 생각 중..." // 응답 대기 메시지 표시
    }

    // TTS 초기화
    override fun onInit(status: Int) {
        if (status == TextToSpeech.SUCCESS) {
            tts.language = Locale.KOREAN
        }
    }

    // 로봇이 음성으로 답변 읽기
    private fun speakAnswer(answer: String) {
        tts.speak(answer, TextToSpeech.QUEUE_FLUSH, null, null)
    }

    override fun onDestroy() {
        if (::tts.isInitialized) {
            tts.stop()
            tts.shutdown()
        }
        super.onDestroy()
    }
}
